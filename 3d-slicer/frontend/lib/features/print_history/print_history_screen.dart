import 'package:flutter/material.dart';

import '../../core/api_client.dart';
import '../../shared/widgets/animated_background.dart';
import '../../shared/widgets/entrance.dart';
import '../../shared/widgets/shimmer.dart';
import '../../theme/app_theme.dart';

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
      final records = (resp.data?['records'] as List?)?.cast<Map<String, dynamic>>() ?? [];
      if (mounted) setState(() => _records = records);
    } catch (_) {
    } finally {
      if (mounted) setState(() => _loading = false);
    }
  }

  Future<void> _delete(String id) async {
    try {
      await apiClient.delete('/api/history/$id');
      setState(() => _records.removeWhere((r) => r['id'] == id));
    } catch (_) {
      _snack('Could not delete record');
    }
  }

  Future<void> _clearAll() async {
    final ok = await showDialog<bool>(
      context: context,
      builder: (_) => AlertDialog(
        title: const Text('Clear all history?'),
        content: const Text('This removes every print record. It cannot be undone.'),
        actions: [
          TextButton(onPressed: () => Navigator.pop(context, false), child: const Text('Cancel')),
          FilledButton(
            style: FilledButton.styleFrom(backgroundColor: AppColors.danger),
            onPressed: () => Navigator.pop(context, true),
            child: const Text('Clear all'),
          ),
        ],
      ),
    );
    if (ok == true) {
      try {
        await apiClient.delete('/api/history');
        setState(() => _records = []);
      } catch (_) {
        _snack('Could not clear history');
      }
    }
  }

  void _snack(String m) {
    if (mounted) ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(m)));
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
              icon: const Icon(Icons.delete_sweep_outlined),
              tooltip: 'Clear all',
              onPressed: _clearAll,
            ),
          const SizedBox(width: 4),
        ],
      ),
      body: AnimatedBackground(
        child: SafeArea(
          top: false,
          child: _loading
              ? _skeleton()
              : _records.isEmpty
                  ? const _EmptyState()
                  : RefreshIndicator(
                      onRefresh: _load,
                      child: ListView.separated(
                        padding: const EdgeInsets.all(16),
                        itemCount: _records.length,
                        separatorBuilder: (_, __) => const SizedBox(height: 10),
                        itemBuilder: (_, i) => FadeSlideIn(
                          index: i,
                          child: _HistoryTile(
                            record: _records[i],
                            onDelete: () => _delete(_records[i]['id'] as String),
                          ),
                        ),
                      ),
                    ),
        ),
      ),
    );
  }

  Widget _skeleton() => ListView.separated(
        padding: const EdgeInsets.all(16),
        itemCount: 5,
        separatorBuilder: (_, __) => const SizedBox(height: 10),
        itemBuilder: (_, __) => Container(
          height: 92,
          decoration: BoxDecoration(
            color: AppColors.surface,
            borderRadius: BorderRadius.circular(AppRadius.md),
            border: Border.all(color: AppColors.surfaceBorder),
          ),
          padding: const EdgeInsets.all(16),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Shimmer.box(width: 160, height: 15),
              const SizedBox(height: 10),
              Shimmer.box(width: 220, height: 11),
            ],
          ),
        ),
      );
}

class _HistoryTile extends StatelessWidget {
  final Map<String, dynamic> record;
  final VoidCallback onDelete;
  const _HistoryTile({required this.record, required this.onDelete});

  @override
  Widget build(BuildContext context) {
    final status = (record['status'] as String? ?? 'unknown');
    final filename = record['filename'] as String? ?? 'Untitled';
    final progress = (record['progress_percent'] as num?)?.toDouble() ?? 0.0;
    final duration = record['duration_seconds'] as int? ?? 0;
    final filament = (record['filament_used_g'] as num?)?.toDouble() ?? 0.0;
    final meta = _StatusMeta.of(status);

    return Dismissible(
      key: ValueKey(record['id']),
      direction: DismissDirection.endToStart,
      onDismissed: (_) => onDelete(),
      background: Container(
        alignment: Alignment.centerRight,
        padding: const EdgeInsets.only(right: 20),
        decoration: BoxDecoration(
          color: AppColors.danger.withOpacity(0.2),
          borderRadius: BorderRadius.circular(AppRadius.md),
        ),
        child: const Icon(Icons.delete_outline, color: AppColors.danger),
      ),
      child: Container(
        padding: const EdgeInsets.all(AppSpace.md),
        decoration: BoxDecoration(
          color: AppColors.surface,
          borderRadius: BorderRadius.circular(AppRadius.md),
          border: Border.all(color: AppColors.surfaceBorder),
        ),
        child: Row(
          children: [
            Container(
              width: 44,
              height: 44,
              decoration: BoxDecoration(
                color: meta.color.withOpacity(0.14),
                borderRadius: BorderRadius.circular(AppRadius.sm),
              ),
              child: Icon(meta.icon, color: meta.color, size: 22),
            ),
            const SizedBox(width: AppSpace.md),
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(filename,
                      maxLines: 1,
                      overflow: TextOverflow.ellipsis,
                      style: const TextStyle(fontWeight: FontWeight.w600)),
                  const SizedBox(height: 4),
                  Wrap(
                    spacing: 12,
                    children: [
                      _stat(Icons.schedule, _fmtDuration(duration)),
                      if (filament > 0) _stat(Icons.bubble_chart_outlined, '${filament.toStringAsFixed(0)} g'),
                      if (status == 'cancelled' || status == 'failed')
                        _stat(Icons.percent, '${progress.toStringAsFixed(0)}%'),
                    ],
                  ),
                ],
              ),
            ),
            const SizedBox(width: 8),
            _Badge(label: meta.label, color: meta.color),
          ],
        ),
      ),
    );
  }

  Widget _stat(IconData icon, String text) => Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(icon, size: 13, color: AppColors.onSurfaceDim),
          const SizedBox(width: 3),
          Text(text, style: const TextStyle(color: AppColors.onSurfaceDim, fontSize: 12.5)),
        ],
      );

  String _fmtDuration(int s) {
    if (s <= 0) return '—';
    final h = s ~/ 3600;
    final m = (s % 3600) ~/ 60;
    if (h > 0) return '${h}h ${m}m';
    return '${m}m';
  }
}

class _Badge extends StatelessWidget {
  final String label;
  final Color color;
  const _Badge({required this.label, required this.color});

  @override
  Widget build(BuildContext context) => Container(
        padding: const EdgeInsets.symmetric(horizontal: 9, vertical: 5),
        decoration: BoxDecoration(
          color: color.withOpacity(0.14),
          borderRadius: BorderRadius.circular(AppRadius.pill),
          border: Border.all(color: color.withOpacity(0.4)),
        ),
        child: Text(label,
            style: TextStyle(color: color, fontSize: 11.5, fontWeight: FontWeight.w700)),
      );
}

class _StatusMeta {
  final String label;
  final Color color;
  final IconData icon;
  const _StatusMeta(this.label, this.color, this.icon);

  static _StatusMeta of(String status) {
    switch (status) {
      case 'completed':
        return const _StatusMeta('Completed', AppColors.success, Icons.check_circle_outline);
      case 'printing':
        return const _StatusMeta('Printing', AppColors.primary, Icons.print_outlined);
      case 'cancelled':
        return const _StatusMeta('Cancelled', AppColors.warning, Icons.cancel_outlined);
      case 'failed':
        return const _StatusMeta('Failed', AppColors.danger, Icons.error_outline);
      default:
        return const _StatusMeta('Unknown', AppColors.onSurfaceDim, Icons.help_outline);
    }
  }
}

class _EmptyState extends StatelessWidget {
  const _EmptyState();
  @override
  Widget build(BuildContext context) => Center(
        child: FadeSlideIn(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Container(
                padding: const EdgeInsets.all(22),
                decoration: BoxDecoration(
                  color: AppColors.surfaceHigh,
                  shape: BoxShape.circle,
                  border: Border.all(color: AppColors.surfaceBorder),
                ),
                child: const Icon(Icons.history, size: 48, color: AppColors.onSurfaceDim),
              ),
              const SizedBox(height: 20),
              Text('No prints yet', style: Theme.of(context).textTheme.titleMedium),
              const SizedBox(height: 8),
              const Text(
                'Completed and cancelled prints will\nappear here automatically.',
                textAlign: TextAlign.center,
                style: TextStyle(color: AppColors.onSurfaceDim, height: 1.4),
              ),
            ],
          ),
        ),
      );
}
