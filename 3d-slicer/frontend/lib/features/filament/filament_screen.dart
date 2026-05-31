import 'package:flutter/material.dart';

import '../../core/api_client.dart';
import '../../shared/widgets/animated_background.dart';
import '../../shared/widgets/entrance.dart';
import '../../theme/app_theme.dart';

class FilamentScreen extends StatefulWidget {
  const FilamentScreen({super.key});

  @override
  State<FilamentScreen> createState() => _FilamentScreenState();
}

class _FilamentScreenState extends State<FilamentScreen> {
  List<Map<String, dynamic>> _spools = [];
  String? _activeId;
  bool _loading = true;

  @override
  void initState() {
    super.initState();
    _load();
  }

  Future<void> _load() async {
    setState(() => _loading = true);
    try {
      final resp = await apiClient.get<Map<String, dynamic>>('/api/filament');
      if (mounted) {
        setState(() {
          _spools = (resp.data?['spools'] as List?)?.cast<Map<String, dynamic>>() ?? [];
          _activeId = resp.data?['active_id'] as String?;
        });
      }
    } catch (_) {
    } finally {
      if (mounted) setState(() => _loading = false);
    }
  }

  Future<void> _activate(String id) async {
    try {
      await apiClient.post('/api/filament/$id/activate');
      await _load();
    } catch (_) {
      _snack('Could not set active spool');
    }
  }

  Future<void> _delete(String id) async {
    try {
      await apiClient.delete('/api/filament/$id');
      await _load();
    } catch (_) {
      _snack('Could not delete spool');
    }
  }

  Future<void> _openEditor([Map<String, dynamic>? spool]) async {
    final result = await showDialog<Map<String, dynamic>>(
      context: context,
      builder: (_) => _SpoolDialog(spool: spool),
    );
    if (result == null) return;
    try {
      if (spool == null) {
        await apiClient.post('/api/filament', data: result);
      } else {
        await apiClient.patch('/api/filament/${spool['id']}', data: result);
      }
      await _load();
    } catch (_) {
      _snack('Could not save spool');
    }
  }

  void _snack(String m) {
    if (mounted) ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(m)));
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Filament'),
        actions: [
          IconButton(icon: const Icon(Icons.refresh), onPressed: _load),
          const SizedBox(width: 4),
        ],
      ),
      floatingActionButton: FloatingActionButton.extended(
        onPressed: () => _openEditor(),
        icon: const Icon(Icons.add),
        label: const Text('Add spool'),
        backgroundColor: AppColors.primary,
        foregroundColor: Colors.white,
      ),
      body: AnimatedBackground(
        child: SafeArea(
          top: false,
          child: _loading
              ? const Center(child: CircularProgressIndicator())
              : _spools.isEmpty
                  ? const _EmptyState()
                  : RefreshIndicator(
                      onRefresh: _load,
                      child: ListView.separated(
                        padding: const EdgeInsets.fromLTRB(16, 16, 16, 90),
                        itemCount: _spools.length,
                        separatorBuilder: (_, __) => const SizedBox(height: 12),
                        itemBuilder: (_, i) => FadeSlideIn(
                          index: i,
                          child: _SpoolCard(
                            spool: _spools[i],
                            isActive: _spools[i]['id'] == _activeId,
                            onActivate: () => _activate(_spools[i]['id'] as String),
                            onEdit: () => _openEditor(_spools[i]),
                            onDelete: () => _delete(_spools[i]['id'] as String),
                          ),
                        ),
                      ),
                    ),
        ),
      ),
    );
  }
}

Color parseHex(String? hex) {
  if (hex == null) return AppColors.primary;
  var h = hex.replaceAll('#', '').trim();
  if (h.length == 6) h = 'FF$h';
  final v = int.tryParse(h, radix: 16);
  return v == null ? AppColors.primary : Color(v);
}

class _SpoolCard extends StatelessWidget {
  final Map<String, dynamic> spool;
  final bool isActive;
  final VoidCallback onActivate;
  final VoidCallback onEdit;
  final VoidCallback onDelete;

  const _SpoolCard({
    required this.spool,
    required this.isActive,
    required this.onActivate,
    required this.onEdit,
    required this.onDelete,
  });

  @override
  Widget build(BuildContext context) {
    final name = spool['name'] as String? ?? 'Spool';
    final material = spool['material'] as String? ?? 'PLA';
    final total = (spool['total_weight_g'] as num?)?.toDouble() ?? 1000;
    final remaining = (spool['remaining_weight_g'] as num?)?.toDouble() ?? 0;
    final color = parseHex(spool['color'] as String?);
    final frac = total > 0 ? (remaining / total).clamp(0.0, 1.0) : 0.0;
    final low = frac < 0.15;

    return Container(
      padding: const EdgeInsets.all(AppSpace.md),
      decoration: BoxDecoration(
        color: AppColors.surface,
        borderRadius: BorderRadius.circular(AppRadius.md),
        border: Border.all(
          color: isActive ? AppColors.primary.withOpacity(0.6) : AppColors.surfaceBorder,
          width: isActive ? 1.6 : 1,
        ),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                width: 38,
                height: 38,
                decoration: BoxDecoration(
                  color: color,
                  shape: BoxShape.circle,
                  border: Border.all(color: Colors.white24, width: 2),
                  boxShadow: [BoxShadow(color: color.withOpacity(0.5), blurRadius: 10)],
                ),
              ),
              const SizedBox(width: AppSpace.md),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Row(
                      children: [
                        Flexible(
                          child: Text(name,
                              maxLines: 1,
                              overflow: TextOverflow.ellipsis,
                              style: const TextStyle(fontWeight: FontWeight.w700, fontSize: 15)),
                        ),
                        if (isActive) ...[
                          const SizedBox(width: 8),
                          Container(
                            padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
                            decoration: BoxDecoration(
                              color: AppColors.primary.withOpacity(0.16),
                              borderRadius: BorderRadius.circular(AppRadius.pill),
                            ),
                            child: const Text('LOADED',
                                style: TextStyle(
                                    color: AppColors.primary,
                                    fontSize: 10,
                                    fontWeight: FontWeight.w800,
                                    letterSpacing: 0.5)),
                          ),
                        ],
                      ],
                    ),
                    Text(material,
                        style: const TextStyle(color: AppColors.onSurfaceDim, fontSize: 13)),
                  ],
                ),
              ),
              PopupMenuButton<String>(
                icon: const Icon(Icons.more_vert, color: AppColors.onSurfaceDim),
                onSelected: (v) {
                  if (v == 'activate') onActivate();
                  if (v == 'edit') onEdit();
                  if (v == 'delete') onDelete();
                },
                itemBuilder: (_) => [
                  if (!isActive)
                    const PopupMenuItem(value: 'activate', child: Text('Set as loaded')),
                  const PopupMenuItem(value: 'edit', child: Text('Edit')),
                  const PopupMenuItem(value: 'delete', child: Text('Delete')),
                ],
              ),
            ],
          ),
          const SizedBox(height: 14),
          TweenAnimationBuilder<double>(
            tween: Tween(begin: 0, end: frac),
            duration: AppMotion.slow,
            curve: AppMotion.curve,
            builder: (_, v, __) => ClipRRect(
              borderRadius: BorderRadius.circular(AppRadius.pill),
              child: Stack(
                children: [
                  Container(height: 10, color: AppColors.surfaceHigh),
                  FractionallySizedBox(
                    widthFactor: v,
                    child: Container(
                      height: 10,
                      decoration: BoxDecoration(
                        color: low ? AppColors.danger : color,
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
          const SizedBox(height: 8),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Text(
                '${remaining.toStringAsFixed(0)} g remaining',
                style: TextStyle(
                  color: low ? AppColors.danger : AppColors.onSurface,
                  fontWeight: FontWeight.w600,
                  fontSize: 13,
                ),
              ),
              Text('${(frac * 100).toStringAsFixed(0)}% of ${total.toStringAsFixed(0)} g',
                  style: const TextStyle(color: AppColors.onSurfaceDim, fontSize: 12.5)),
            ],
          ),
        ],
      ),
    );
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
                child: const Icon(Icons.bubble_chart_outlined, size: 48, color: AppColors.onSurfaceDim),
              ),
              const SizedBox(height: 20),
              Text('No spools yet', style: Theme.of(context).textTheme.titleMedium),
              const SizedBox(height: 8),
              const Text(
                'Add a spool to track remaining filament.\nThe loaded spool is deducted after each print.',
                textAlign: TextAlign.center,
                style: TextStyle(color: AppColors.onSurfaceDim, height: 1.4),
              ),
            ],
          ),
        ),
      );
}

class _SpoolDialog extends StatefulWidget {
  final Map<String, dynamic>? spool;
  const _SpoolDialog({this.spool});

  @override
  State<_SpoolDialog> createState() => _SpoolDialogState();
}

class _SpoolDialogState extends State<_SpoolDialog> {
  late final TextEditingController _name;
  late final TextEditingController _total;
  late final TextEditingController _remaining;
  late String _material;
  late String _color;

  static const _materials = ['PLA', 'PETG', 'ABS', 'ASA', 'TPU', 'PC', 'Nylon'];
  static const _palette = [
    '#E85D04', '#F87171', '#FBBF24', '#34D399',
    '#38BDF8', '#818CF8', '#F472B6', '#E7E9EE', '#16181F',
  ];

  @override
  void initState() {
    super.initState();
    final s = widget.spool;
    _name = TextEditingController(text: s?['name'] as String? ?? '');
    _total = TextEditingController(text: (s?['total_weight_g'] as num?)?.toString() ?? '1000');
    _remaining = TextEditingController(
        text: (s?['remaining_weight_g'] as num?)?.toString() ??
            (s?['total_weight_g'] as num?)?.toString() ?? '1000');
    _material = s?['material'] as String? ?? 'PLA';
    _color = s?['color'] as String? ?? _palette.first;
  }

  @override
  void dispose() {
    _name.dispose();
    _total.dispose();
    _remaining.dispose();
    super.dispose();
  }

  void _save() {
    final name = _name.text.trim();
    if (name.isEmpty) return;
    final total = double.tryParse(_total.text.trim()) ?? 1000;
    final remaining = double.tryParse(_remaining.text.trim()) ?? total;
    Navigator.pop(context, {
      'name': name,
      'material': _material,
      'color': _color,
      'total_weight_g': total,
      'remaining_weight_g': remaining,
    });
  }

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: Text(widget.spool == null ? 'Add spool' : 'Edit spool'),
      content: SingleChildScrollView(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            TextField(
              controller: _name,
              autofocus: true,
              decoration: const InputDecoration(labelText: 'Name', hintText: 'Prusament Galaxy Black'),
            ),
            const SizedBox(height: 14),
            DropdownButtonFormField<String>(
              value: _material,
              decoration: const InputDecoration(labelText: 'Material'),
              items: _materials.map((m) => DropdownMenuItem(value: m, child: Text(m))).toList(),
              onChanged: (v) => setState(() => _material = v ?? _material),
            ),
            const SizedBox(height: 14),
            Row(
              children: [
                Expanded(
                  child: TextField(
                    controller: _total,
                    keyboardType: TextInputType.number,
                    decoration: const InputDecoration(labelText: 'Total (g)'),
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: TextField(
                    controller: _remaining,
                    keyboardType: TextInputType.number,
                    decoration: const InputDecoration(labelText: 'Remaining (g)'),
                  ),
                ),
              ],
            ),
            const SizedBox(height: 16),
            const Text('Colour', style: TextStyle(color: AppColors.onSurfaceDim, fontSize: 13)),
            const SizedBox(height: 8),
            Wrap(
              spacing: 10,
              runSpacing: 10,
              children: _palette.map((hex) {
                final selected = hex == _color;
                return GestureDetector(
                  onTap: () => setState(() => _color = hex),
                  child: Container(
                    width: 30,
                    height: 30,
                    decoration: BoxDecoration(
                      color: parseHex(hex),
                      shape: BoxShape.circle,
                      border: Border.all(
                        color: selected ? Colors.white : Colors.white24,
                        width: selected ? 3 : 1,
                      ),
                    ),
                    child: selected
                        ? const Icon(Icons.check, size: 16, color: Colors.black)
                        : null,
                  ),
                );
              }).toList(),
            ),
          ],
        ),
      ),
      actions: [
        TextButton(onPressed: () => Navigator.pop(context), child: const Text('Cancel')),
        FilledButton(onPressed: _save, child: const Text('Save')),
      ],
    );
  }
}
