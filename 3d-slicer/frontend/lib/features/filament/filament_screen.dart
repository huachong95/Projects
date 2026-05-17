import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import '../../core/api_client.dart';

class FilamentScreen extends StatefulWidget {
  const FilamentScreen({super.key});

  @override
  State<FilamentScreen> createState() => _FilamentScreenState();
}

class _FilamentScreenState extends State<FilamentScreen> {
  List<Map<String, dynamic>> _spools = [];
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
      final spools =
          (resp.data?['spools'] as List?)?.cast<Map<String, dynamic>>() ?? [];
      setState(() => _spools = spools);
    } catch (_) {
    } finally {
      setState(() => _loading = false);
    }
  }

  Future<void> _delete(String id) async {
    try {
      await apiClient.delete('/api/filament/$id');
      setState(() => _spools.removeWhere((s) => s['spool_id'] == id));
      _snack('Spool deleted');
    } catch (e) {
      _snack('Delete failed: $e');
    }
  }

  Future<void> _showAddEdit([Map<String, dynamic>? existing]) async {
    final result = await showDialog<bool>(
      context: context,
      builder: (_) => _SpoolDialog(existing: existing),
    );
    if (result == true) _load();
  }

  void _snack(String msg) {
    if (mounted) ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(msg)));
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Filament Manager'),
        actions: [
          IconButton(icon: const Icon(Icons.refresh), onPressed: _load),
        ],
      ),
      floatingActionButton: FloatingActionButton.extended(
        onPressed: () => _showAddEdit(),
        icon: const Icon(Icons.add),
        label: const Text('Add spool'),
      ),
      body: _loading
          ? const Center(child: CircularProgressIndicator())
          : _spools.isEmpty
              ? Center(
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Icon(Icons.straighten, size: 64, color: Colors.grey[600]),
                      const SizedBox(height: 16),
                      const Text('No spools yet'),
                      const SizedBox(height: 8),
                      TextButton(
                        onPressed: () => _showAddEdit(),
                        child: const Text('Add your first spool'),
                      ),
                    ],
                  ),
                )
              : ListView.builder(
                  padding: const EdgeInsets.fromLTRB(12, 12, 12, 80),
                  itemCount: _spools.length,
                  itemBuilder: (_, i) => _SpoolCard(
                    spool: _spools[i],
                    onEdit: () => _showAddEdit(_spools[i]),
                    onDelete: () => _delete(_spools[i]['spool_id'] as String),
                  ),
                ),
    );
  }
}

class _SpoolCard extends StatelessWidget {
  final Map<String, dynamic> spool;
  final VoidCallback onEdit;
  final VoidCallback onDelete;

  const _SpoolCard({required this.spool, required this.onEdit, required this.onDelete});

  Color _hexColor(String hex) {
    try {
      return Color(int.parse(hex.replaceFirst('#', '0xFF')));
    } catch (_) {
      return Colors.grey;
    }
  }

  @override
  Widget build(BuildContext context) {
    final name = spool['name'] as String? ?? 'Unknown';
    final material = spool['material'] as String? ?? '';
    final vendor = spool['vendor'] as String? ?? '';
    final colorHex = spool['color_hex'] as String? ?? '#888888';
    final remainingPct = (spool['remaining_percent'] as num?)?.toDouble() ?? 0;
    final remainingG = (spool['remaining_weight_g'] as num?)?.toDouble() ?? 0;
    final initialG = (spool['initial_weight_g'] as num?)?.toDouble() ?? 1000;
    final nozzleTemp = spool['nozzle_temp_c'] as int? ?? 0;
    final bedTemp = spool['bed_temp_c'] as int? ?? 0;

    final color = _hexColor(colorHex);
    final pct = remainingPct.clamp(0, 100) / 100;
    final barColor = pct > 0.3 ? Colors.green : pct > 0.1 ? Colors.orange : Colors.red;

    return Card(
      margin: const EdgeInsets.only(bottom: 10),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  width: 36,
                  height: 36,
                  decoration: BoxDecoration(
                    color: color,
                    shape: BoxShape.circle,
                    border: Border.all(color: Colors.white24, width: 2),
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(name, style: Theme.of(context).textTheme.titleSmall),
                      Text(
                        [material, if (vendor.isNotEmpty) vendor].join(' · '),
                        style: Theme.of(context).textTheme.bodySmall?.copyWith(
                              color: Theme.of(context).colorScheme.onSurfaceVariant,
                            ),
                      ),
                    ],
                  ),
                ),
                IconButton(icon: const Icon(Icons.edit_outlined, size: 20), onPressed: onEdit),
                IconButton(
                  icon: const Icon(Icons.delete_outline, size: 20),
                  onPressed: onDelete,
                ),
              ],
            ),
            const SizedBox(height: 12),
            Row(
              children: [
                Expanded(
                  child: ClipRRect(
                    borderRadius: BorderRadius.circular(4),
                    child: LinearProgressIndicator(
                      value: pct,
                      color: barColor,
                      backgroundColor: Colors.white12,
                      minHeight: 8,
                    ),
                  ),
                ),
                const SizedBox(width: 10),
                Text(
                  '${remainingG.toStringAsFixed(0)} g',
                  style: Theme.of(context).textTheme.bodySmall,
                ),
                Text(
                  ' / ${initialG.toStringAsFixed(0)} g',
                  style: Theme.of(context).textTheme.bodySmall?.copyWith(
                        color: Theme.of(context).colorScheme.onSurfaceVariant,
                      ),
                ),
              ],
            ),
            const SizedBox(height: 8),
            Wrap(
              spacing: 16,
              children: [
                _TempChip(icon: Icons.whatshot, label: '${nozzleTemp}°C nozzle'),
                _TempChip(icon: Icons.bed, label: '${bedTemp}°C bed'),
                _TempChip(
                  icon: Icons.donut_large,
                  label: '${remainingPct.toStringAsFixed(0)}% remaining',
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}

class _TempChip extends StatelessWidget {
  final IconData icon;
  final String label;
  const _TempChip({required this.icon, required this.label});

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

class _SpoolDialog extends StatefulWidget {
  final Map<String, dynamic>? existing;
  const _SpoolDialog({this.existing});

  @override
  State<_SpoolDialog> createState() => _SpoolDialogState();
}

class _SpoolDialogState extends State<_SpoolDialog> {
  final _form = GlobalKey<FormState>();
  late final TextEditingController _name;
  late final TextEditingController _material;
  late final TextEditingController _vendor;
  late final TextEditingController _colorHex;
  late final TextEditingController _initialWeight;
  late final TextEditingController _nozzleTemp;
  late final TextEditingController _bedTemp;
  late final TextEditingController _notes;
  bool _saving = false;

  @override
  void initState() {
    super.initState();
    final e = widget.existing;
    _name = TextEditingController(text: e?['name'] as String? ?? '');
    _material = TextEditingController(text: e?['material'] as String? ?? 'PLA');
    _vendor = TextEditingController(text: e?['vendor'] as String? ?? '');
    _colorHex = TextEditingController(text: e?['color_hex'] as String? ?? '#FFFFFF');
    _initialWeight = TextEditingController(
        text: ((e?['initial_weight_g'] as num?)?.toStringAsFixed(0)) ?? '1000');
    _nozzleTemp =
        TextEditingController(text: (e?['nozzle_temp_c'] as int?)?.toString() ?? '215');
    _bedTemp =
        TextEditingController(text: (e?['bed_temp_c'] as int?)?.toString() ?? '60');
    _notes = TextEditingController(text: e?['notes'] as String? ?? '');
  }

  @override
  void dispose() {
    for (final c in [_name, _material, _vendor, _colorHex, _initialWeight, _nozzleTemp, _bedTemp, _notes]) {
      c.dispose();
    }
    super.dispose();
  }

  Future<void> _save() async {
    if (!_form.currentState!.validate()) return;
    setState(() => _saving = true);
    final body = {
      'name': _name.text.trim(),
      'material': _material.text.trim(),
      'vendor': _vendor.text.trim(),
      'color_hex': _colorHex.text.trim(),
      'initial_weight_g': double.tryParse(_initialWeight.text) ?? 1000,
      'nozzle_temp_c': int.tryParse(_nozzleTemp.text) ?? 215,
      'bed_temp_c': int.tryParse(_bedTemp.text) ?? 60,
      'notes': _notes.text.trim(),
    };
    try {
      final existing = widget.existing;
      if (existing != null) {
        await apiClient.patch('/api/filament/${existing['spool_id']}', data: body);
      } else {
        await apiClient.post('/api/filament', data: body);
      }
      if (mounted) Navigator.pop(context, true);
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(context)
            .showSnackBar(SnackBar(content: Text('Save failed: $e')));
      }
    } finally {
      if (mounted) setState(() => _saving = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    final isEdit = widget.existing != null;
    return AlertDialog(
      title: Text(isEdit ? 'Edit spool' : 'Add spool'),
      content: SizedBox(
        width: 360,
        child: Form(
          key: _form,
          child: SingleChildScrollView(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                _field(_name, 'Name', required: true),
                const SizedBox(height: 12),
                _field(_material, 'Material (PLA, PETG, ABS…)', required: true),
                const SizedBox(height: 12),
                _field(_vendor, 'Vendor'),
                const SizedBox(height: 12),
                _field(_colorHex, 'Color hex (#RRGGBB)',
                    validator: (v) => (v == null || !RegExp(r'^#[0-9A-Fa-f]{6}$').hasMatch(v))
                        ? 'Enter a valid hex color'
                        : null),
                const SizedBox(height: 12),
                _field(_initialWeight, 'Initial weight (g)',
                    keyboardType: TextInputType.number,
                    inputFormatters: [FilteringTextInputFormatter.digitsOnly]),
                const SizedBox(height: 12),
                Row(
                  children: [
                    Expanded(
                      child: _field(_nozzleTemp, 'Nozzle °C',
                          keyboardType: TextInputType.number,
                          inputFormatters: [FilteringTextInputFormatter.digitsOnly]),
                    ),
                    const SizedBox(width: 12),
                    Expanded(
                      child: _field(_bedTemp, 'Bed °C',
                          keyboardType: TextInputType.number,
                          inputFormatters: [FilteringTextInputFormatter.digitsOnly]),
                    ),
                  ],
                ),
                const SizedBox(height: 12),
                _field(_notes, 'Notes', maxLines: 2),
              ],
            ),
          ),
        ),
      ),
      actions: [
        TextButton(
          onPressed: _saving ? null : () => Navigator.pop(context, false),
          child: const Text('Cancel'),
        ),
        FilledButton(
          onPressed: _saving ? null : _save,
          child: _saving
              ? const SizedBox(width: 16, height: 16, child: CircularProgressIndicator(strokeWidth: 2))
              : Text(isEdit ? 'Save' : 'Add'),
        ),
      ],
    );
  }

  Widget _field(
    TextEditingController controller,
    String label, {
    bool required = false,
    String? Function(String?)? validator,
    TextInputType? keyboardType,
    List<TextInputFormatter>? inputFormatters,
    int maxLines = 1,
  }) {
    return TextFormField(
      controller: controller,
      keyboardType: keyboardType,
      inputFormatters: inputFormatters,
      maxLines: maxLines,
      decoration: InputDecoration(
        labelText: label,
        border: const OutlineInputBorder(),
        isDense: true,
      ),
      validator: validator ??
          (required
              ? (v) => (v == null || v.trim().isEmpty) ? 'Required' : null
              : null),
    );
  }
}
