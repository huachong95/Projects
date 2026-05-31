import 'package:desktop_drop/desktop_drop.dart';
import 'package:dio/dio.dart';
import 'package:file_picker/file_picker.dart';
import 'package:flutter/material.dart';
import 'package:go_router/go_router.dart';

import '../../core/api_client.dart';
import '../../shared/widgets/animated_background.dart';
import '../../shared/widgets/entrance.dart';
import '../../theme/app_theme.dart';

class ModelImportScreen extends StatefulWidget {
  const ModelImportScreen({super.key});

  @override
  State<ModelImportScreen> createState() => _ModelImportScreenState();
}

class _ModelImportScreenState extends State<ModelImportScreen> {
  bool _loading = false;
  bool _dragging = false;
  String? _errorMsg;
  Map<String, dynamic>? _meshInfo;

  static const _allowed = {'stl', '3mf', 'obj'};

  Future<void> _pickFile() async {
    final result = await FilePicker.platform.pickFiles(
      type: FileType.custom,
      allowedExtensions: _allowed.toList(),
    );
    if (result == null || result.files.isEmpty) return;
    final file = result.files.first;
    if (file.path == null) return;
    await _upload(file.path!, file.name);
  }

  Future<void> _onDrop(DropDoneDetails detail) async {
    if (detail.files.isEmpty) return;
    final f = detail.files.first;
    final ext = f.name.split('.').last.toLowerCase();
    if (!_allowed.contains(ext)) {
      setState(() => _errorMsg = 'Unsupported file type ".$ext". Use STL, 3MF or OBJ.');
      return;
    }
    await _upload(f.path, f.name);
  }

  Future<void> _upload(String path, String filename) async {
    setState(() {
      _loading = true;
      _errorMsg = null;
      _meshInfo = null;
    });
    try {
      final resp = await apiClient.uploadFile<Map<String, dynamic>>(
        '/api/mesh/upload',
        path,
        'file',
        filename: filename,
      );
      setState(() => _meshInfo = resp.data);
    } on DioException catch (e) {
      setState(() => _errorMsg = e.response?.data?.toString() ?? e.message);
    } finally {
      setState(() => _loading = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Import Model')),
      body: AnimatedBackground(
        child: SafeArea(
          top: false,
          child: SingleChildScrollView(
            padding: const EdgeInsets.all(24),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                FadeSlideIn(
                  child: DropTarget(
                    onDragEntered: (_) => setState(() => _dragging = true),
                    onDragExited: (_) => setState(() => _dragging = false),
                    onDragDone: (d) {
                      setState(() => _dragging = false);
                      _onDrop(d);
                    },
                    child: _DropZone(
                      loading: _loading,
                      dragging: _dragging,
                      onTap: _pickFile,
                    ),
                  ),
                ),
                if (_errorMsg != null) ...[
                  const SizedBox(height: AppSpace.md),
                  _ErrorRow(_errorMsg!),
                ],
                if (_meshInfo != null) ...[
                  const SizedBox(height: AppSpace.lg),
                  FadeSlideIn(child: _MeshInfoCard(info: _meshInfo!)),
                  const SizedBox(height: AppSpace.md),
                  FadeSlideIn(
                    index: 1,
                    child: SizedBox(
                      width: double.infinity,
                      child: FilledButton.icon(
                        icon: const Icon(Icons.tune),
                        label: const Text('Continue to slice settings'),
                        onPressed: () => context.push('/settings/${_meshInfo!['job_id']}'),
                      ),
                    ),
                  ),
                ],
              ],
            ),
          ),
        ),
      ),
    );
  }
}

class _DropZone extends StatelessWidget {
  final bool loading;
  final bool dragging;
  final VoidCallback onTap;

  const _DropZone({required this.loading, required this.dragging, required this.onTap});

  @override
  Widget build(BuildContext context) {
    final accent = dragging ? AppColors.primaryBright : AppColors.surfaceBorder;
    return GestureDetector(
      onTap: loading ? null : onTap,
      child: AnimatedContainer(
        duration: AppMotion.fast,
        height: 220,
        decoration: BoxDecoration(
          color: dragging ? AppColors.primary.withOpacity(0.08) : AppColors.surface.withOpacity(0.6),
          border: Border.all(color: accent, width: dragging ? 2.5 : 1.5),
          borderRadius: BorderRadius.circular(AppRadius.lg),
          boxShadow: dragging
              ? [BoxShadow(color: AppColors.primary.withOpacity(0.25), blurRadius: 28, spreadRadius: -4)]
              : const [],
        ),
        child: Center(
          child: loading
              ? const Column(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    CircularProgressIndicator(),
                    SizedBox(height: 16),
                    Text('Uploading & analysing mesh…'),
                  ],
                )
              : Column(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    AnimatedScale(
                      scale: dragging ? 1.15 : 1.0,
                      duration: AppMotion.fast,
                      curve: AppMotion.emphasis,
                      child: Container(
                        padding: const EdgeInsets.all(18),
                        decoration: BoxDecoration(
                          color: AppColors.primary.withOpacity(0.14),
                          shape: BoxShape.circle,
                        ),
                        child: Icon(
                          dragging ? Icons.file_download_outlined : Icons.upload_file_outlined,
                          size: 36,
                          color: AppColors.primaryBright,
                        ),
                      ),
                    ),
                    const SizedBox(height: 16),
                    Text(
                      dragging ? 'Release to import' : 'Drop a model here, or click to browse',
                      style: const TextStyle(fontWeight: FontWeight.w600, fontSize: 15),
                    ),
                    const SizedBox(height: 6),
                    const Text('Supports STL · 3MF · OBJ',
                        style: TextStyle(color: AppColors.onSurfaceDim, fontSize: 12.5)),
                  ],
                ),
        ),
      ),
    );
  }
}

class _ErrorRow extends StatelessWidget {
  final String message;
  const _ErrorRow(this.message);
  @override
  Widget build(BuildContext context) => Container(
        padding: const EdgeInsets.all(12),
        decoration: BoxDecoration(
          color: AppColors.danger.withOpacity(0.12),
          borderRadius: BorderRadius.circular(AppRadius.sm),
          border: Border.all(color: AppColors.danger.withOpacity(0.4)),
        ),
        child: Row(
          children: [
            const Icon(Icons.error_outline, color: AppColors.danger, size: 18),
            const SizedBox(width: 8),
            Expanded(child: Text(message)),
          ],
        ),
      );
}

class _MeshInfoCard extends StatelessWidget {
  final Map<String, dynamic> info;
  const _MeshInfoCard({required this.info});

  @override
  Widget build(BuildContext context) {
    final dims = info['dimensions_mm'] as Map<String, dynamic>? ?? {};
    return Container(
      padding: const EdgeInsets.all(AppSpace.md),
      decoration: BoxDecoration(
        color: AppColors.surface,
        borderRadius: BorderRadius.circular(AppRadius.md),
        border: Border.all(color: AppColors.surfaceBorder),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              const Icon(Icons.check_circle, color: AppColors.success, size: 20),
              const SizedBox(width: 8),
              Text('Model loaded', style: Theme.of(context).textTheme.titleMedium),
            ],
          ),
          const Divider(),
          _Row('File', info['filename']?.toString() ?? ''),
          _Row('Triangles', _fmt(info['triangle_count'])),
          _Row('Size (mm)', 'X ${_d(dims['x'])}  Y ${_d(dims['y'])}  Z ${_d(dims['z'])}'),
          _Row('Volume', '${_d(info['volume_cm3'])} cm³'),
          if (info['was_repaired'] == true)
            const Padding(
              padding: EdgeInsets.only(top: 8),
              child: Row(children: [
                Icon(Icons.build, size: 16, color: AppColors.warning),
                SizedBox(width: 6),
                Text('Mesh was auto-repaired', style: TextStyle(color: AppColors.warning)),
              ]),
            ),
        ],
      ),
    );
  }

  String _fmt(dynamic v) => v?.toString() ?? '—';
  String _d(dynamic v) => v != null ? (v as num).toStringAsFixed(1) : '—';
}

class _Row extends StatelessWidget {
  final String label;
  final String value;
  const _Row(this.label, this.value);

  @override
  Widget build(BuildContext context) => Padding(
        padding: const EdgeInsets.symmetric(vertical: 5),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Text(label, style: const TextStyle(color: AppColors.onSurfaceDim, fontSize: 13)),
            Text(value, style: const TextStyle(fontWeight: FontWeight.w600)),
          ],
        ),
      );
}
