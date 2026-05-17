import 'package:flutter/material.dart';

import '../../core/api_client.dart';

class PrintHistoryScreen extends StatefulWidget {
  const PrintHistoryScreen({super.key});

  @override
  State<PrintHistoryScreen> createState() => _PrintHistoryScreenState();
}

class _PrintHistoryScreenState extends State<PrintHistoryScreen> {
  List<Map<String, dynamic>> _records = [];
  bool _loading = true;

  @override
  void initState() {
    super.initState();
    _load();
  }

  Future<void> _load() async {
    setState(() => _loading = true);
    try {
      final resp = await apiClient.get<Map<String, dynamic>>('/api/history');
      final records =
          (resp.data?['records'] as List?)?.cast<Map<String, dynamic>>() ?? [];
      setState(() => _records = records);
    } catch (_) {
    } finally {
      setState(() => _loading = false);
    }
  }

  Future<void> _delete(String id) async {
    try {
      await apiClient.delete('/api/history/$id');
      setState(() => _records.removeWhere((r) => r['record_id'] == id));
    } catch (e) {
      _snack('Delete failed: $e');
    }
  }

  Future<void> _clearAll() async {
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (_) => AlertDialog(
        title: const Text('Clear history?'),
        content: const Text('All print history records will be permanently deleted.'),
        actions: [
          TextButton(onPressed: () => Navigator.pop(context, false), child: const Text('Cancel')),
          FilledButton(
            style: FilledButton.styleFrom(backgroundColor: Colors.red),
            onPressed: () => Navigator.pop(context, true),
            child: const Text('Clear all'),
          ),
        ],
      ),
    );
    if (confirmed != true) return;
    try {
      await apiClient.delete('/api/history');
      setState(() => _records = []);
      _snack('History cleared');
    } catch (e) {
      _snack('Failed: $e');
    }
  }

  void _snack(String msg) {
    if (mounted) ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(msg)));
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Print History'),
        actions: [
          IconButton(icon: const Icon(Icons.refresh), onPressed: _load),
          if (_records.isNotEmpty)
            IconButton(
              icon: const Icon(Icons.delete_sweep),
              tooltip: 'Clear all',
              onPressed: _clearAll,
            ),
        ],
      ),
      body: _loading
          ? const Center(child: CircularProgressIndicator())
          : _records.isEmpty
              ? const Center(
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Icon(Icons.history, size: 64, color: Colors.grey),
                      SizedBox(height: 16),
                      Text('No prints yet'),
                      SizedBox(height: 8),
                      Text('Completed prints will appear here.',
                          style: TextStyle(color: Colors.grey)),
                    ],
                  ),
                )
              : ListView.builder(
                  padding: const EdgeInsets.all(12),
                  itemCount: _records.length,
                  itemBuilder: (_, i) => _RecordCard(
                    record: _records[i],
                    onDelete: () => _delete(_records[i]['record_id'] as String),
                  ),
                ),
    );
  }
}

class _RecordCard extends StatelessWidget {
  final Map<String, dynamic> record;
  final VoidCallback onDelete;

  const _RecordCard({required this.record, required this.onDelete});

  @override
  Widget build(BuildContext context) {
    final status = record['status'] as String? ?? 'unknown';
    final filename = record['filename'] as String? ?? '—';
    final layers = record['layer_count'] as int? ?? 0;
    final filamentG = (record['filament_used_g'] as num?)?.toDouble() ?? 0;
    final durationSecs = record['duration_seconds'] as int?;
    final startedAt = record['started_at'] as num?;

    final statusColor = switch (status) {
      'success' => Colors.greenAccent,
      'failed' => Colors.redAccent,
      'cancelled' => Colors.orangeAccent,
      'running' => Colors.blueAccent,
      _ => Colors.grey,
    };
    final statusIcon = switch (status) {
      'success' => Icons.check_circle,
      'failed' => Icons.error,
      'cancelled' => Icons.cancel,
      'running' => Icons.print,
      _ => Icons.help,
    };

    return Card(
      margin: const EdgeInsets.only(bottom: 8),
      child: ListTile(
        leading: Icon(statusIcon, color: statusColor, size: 28),
        title: Text(filename, overflow: TextOverflow.ellipsis),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const SizedBox(height: 2),
            Wrap(
              spacing: 12,
              children: [
                if (durationSecs != null)
                  _Stat(Icons.access_time, _fmtDuration(durationSecs)),
                if (layers > 0) _Stat(Icons.layers, '$layers layers'),
                if (filamentG > 0)
                  _Stat(Icons.straighten, '${filamentG.toStringAsFixed(1)} g'),
                if (startedAt != null)
                  _Stat(Icons.calendar_today,
                      _fmtDate(DateTime.fromMillisecondsSinceEpoch(
                          (startedAt * 1000).toInt()))),
              ],
            ),
          ],
        ),
        trailing: IconButton(
          icon: const Icon(Icons.delete_outline, size: 20),
          onPressed: onDelete,
        ),
        isThreeLine: true,
      ),
    );
  }

  static String _fmtDuration(int seconds) {
    final h = seconds ~/ 3600;
    final m = (seconds % 3600) ~/ 60;
    final s = seconds % 60;
    if (h > 0) return '${h}h ${m}m';
    if (m > 0) return '${m}m ${s}s';
    return '${s}s';
  }

  static String _fmtDate(DateTime dt) =>
      '${dt.month}/${dt.day} ${dt.hour}:${dt.minute.toString().padLeft(2, '0')}';
}

class _Stat extends StatelessWidget {
  final IconData icon;
  final String label;
  const _Stat(this.icon, this.label);

  @override
  Widget build(BuildContext context) => Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(icon, size: 12, color: Colors.grey),
          const SizedBox(width: 3),
          Text(label, style: Theme.of(context).textTheme.bodySmall),
        ],
      );
}
