#!/usr/bin/env bash
# =====================================================================
# SCV 운영 DB 구성 (2차) — 관제 PC(Victus)에서 실행
#   1) PostgreSQL 16 컨테이너 robotdb4_postgresql  (영구 데이터, 볼륨 robotdata4_postgresql)
#   2) 스키마(01) · 초기값(02) 적용, 규칙 검증(03)은 --verify 일 때만
#   3) Redis 8 컨테이너 robotdb4_redis              (실시간 데이터, 볼륨 robotdata4_redis)
#
#   사용:  ./setup_db.sh            # 컨테이너 생성 + 스키마·초기값 적용
#          ./setup_db.sh --verify   # 위 + 03_verify.sql (모두 ROLLBACK, 데이터 남지 않음)
#   여러 번 실행해도 됨: 컨테이너·볼륨이 있으면 그대로 쓰고, SQL은 IF NOT EXISTS / ON CONFLICT
# =====================================================================
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
SQL_DIR="$HERE/../sql"

PG_NAME=robotdb4_postgresql
PG_VOLUME=robotdata4_postgresql
PG_DB=robotdb4_postgresql
PG_USER=rokey
PG_PASSWORD=rokey

RD_NAME=robotdb4_redis
RD_VOLUME=robotdata4_redis

VERIFY=0
[[ "${1:-}" == "--verify" ]] && VERIFY=1

exists() { docker ps -a --format '{{.Names}}' | grep -qx "$1"; }
running() { docker ps --format '{{.Names}}' | grep -qx "$1"; }

# ---------------------------------------------------------------------
# 1. PostgreSQL 16
# ---------------------------------------------------------------------
echo "[1/4] PostgreSQL 컨테이너"
docker pull postgres:16
docker volume create "$PG_VOLUME" >/dev/null
if ! exists "$PG_NAME"; then
    docker run -d --name "$PG_NAME" \
        -e POSTGRES_DB="$PG_DB" \
        -e POSTGRES_USER="$PG_USER" \
        -e POSTGRES_PASSWORD="$PG_PASSWORD" \
        -p 5432:5432 \
        -v "$PG_VOLUME":/var/lib/postgresql/data \
        postgres:16
elif ! running "$PG_NAME"; then
    docker start "$PG_NAME" >/dev/null
fi

# 첫 실행 때 이미지가 임시 서버로 초기화한 뒤 다시 시작하므로, TCP 접속이 될 때까지 대기
echo -n "      접속 대기"
for _ in $(seq 1 60); do
    if docker exec "$PG_NAME" pg_isready -h 127.0.0.1 -U "$PG_USER" -d "$PG_DB" >/dev/null 2>&1; then
        echo " → 준비됨"; break
    fi
    echo -n "."; sleep 1
done
docker exec "$PG_NAME" pg_isready -h 127.0.0.1 -U "$PG_USER" -d "$PG_DB" >/dev/null

psql_run() {
    docker exec -i "$PG_NAME" psql -h 127.0.0.1 -U "$PG_USER" -d "$PG_DB" -v ON_ERROR_STOP=1 -q "$@"
}

# ---------------------------------------------------------------------
# 2. 스키마 · 초기값
# ---------------------------------------------------------------------
echo "[2/4] 스키마 적용 (01_schema.sql)"
psql_run < "$SQL_DIR/01_schema.sql"
echo "      초기값 적용 (02_seed.sql)"
psql_run < "$SQL_DIR/02_seed.sql"

if [[ $VERIFY -eq 1 ]]; then
    echo "[3/4] 규칙 검증 (03_verify.sql)"
    psql_run < "$SQL_DIR/03_verify.sql"
else
    echo "[3/4] 규칙 검증 건너뜀 (--verify로 실행)"
fi

# ---------------------------------------------------------------------
# 3. Redis 8
# ---------------------------------------------------------------------
echo "[4/4] Redis 컨테이너"
docker pull redis:8
docker volume create "$RD_VOLUME" >/dev/null
if ! exists "$RD_NAME"; then
    docker run -d --name "$RD_NAME" \
        -p 6379:6379 \
        -v "$RD_VOLUME":/data \
        -v "$HERE/redis.conf":/usr/local/etc/redis/redis.conf:ro \
        redis:8 redis-server /usr/local/etc/redis/redis.conf
elif ! running "$RD_NAME"; then
    docker start "$RD_NAME" >/dev/null
fi
sleep 1
docker exec "$RD_NAME" redis-cli --user rokey --pass rokey --no-auth-warning ping

# ---------------------------------------------------------------------
echo
echo "완료"
docker exec "$PG_NAME" psql -h 127.0.0.1 -U "$PG_USER" -d "$PG_DB" -Atc \
    "SELECT '  PostgreSQL 테이블 ' || count(*) || '개' FROM information_schema.tables WHERE table_schema = 'public'"
echo "  PostgreSQL  postgresql://$PG_USER:****@<관제PC>:5432/$PG_DB"
echo "  Redis       redis://rokey:****@<관제PC>:6379/0"
