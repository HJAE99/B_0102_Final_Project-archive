"""Read-only presentation DTO: committed decisions and live samples stay distinct."""
def merge_readings(samples, committed, min_samples=3):
    rows = {(r['cell_id'], r['ap_id']): dict(r, statistic='live_median', state='unclassified',
                confidence='ok' if r['n'] >= min_samples else 'low', interpolated=False) for r in samples}
    for r in committed:
        key = (r['cell_id'], r['ap_id'])
        original = rows.get(key, {})
        rows[key] = dict(original, **r, statistic='committed_median')
    for r in rows.values():
        r['misses'] = int(r.get('misses') or 0)
        r['detection'] = 'detected' if r['n'] > 0 else 'not_detected' if r['misses'] else 'unmeasured'
        # A sentinel RSSI from an undetected sample is never a usable value.
        if not r['n']:
            r['median'] = None
    return list(rows.values())


def read_readings(conn, run_id, map_id, min_samples):
    samples = conn.execute(
        'SELECT n.cell_id, n.ap_id, count(*) FILTER (WHERE n.detected)::int AS n, '
        'count(*) FILTER (WHERE NOT n.detected)::int AS misses, '
        'percentile_cont(0.5) WITHIN GROUP (ORDER BY n.rssi_dbm) FILTER (WHERE n.detected) AS median, '
        'avg(n.rssi_dbm) FILTER (WHERE n.detected) AS mean, max(n.received_at) AS updated_at '
        'FROM net_samples n JOIN grid_cells g USING (cell_id) JOIN aps a USING (ap_id) '
        'WHERE n.run_id = %s AND n.pose_ok AND g.map_id = %s AND a.map_id = %s '
        'GROUP BY n.cell_id, n.ap_id', (run_id, map_id, map_id)).fetchall()
    committed = conn.execute(
        'SELECT s.cell_id, s.ap_id, s.n, s.median, s.mean, s.confidence, s.state, s.interpolated, '
        's.computed_at AS updated_at FROM grid_stats s JOIN grid_cells g USING (cell_id) '
        'JOIN aps a USING (ap_id) WHERE s.run_id = %s AND g.map_id = %s AND a.map_id = %s',
        (run_id, map_id, map_id)).fetchall()
    return merge_readings(samples, committed, min_samples)
