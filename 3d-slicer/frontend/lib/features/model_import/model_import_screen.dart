import 'package:desktop_drop/desktop_drop.dart';
import 'package:dio/dio.dart';
import 'package:file_picker/file_picker.dart';
import 'package:flutter/material.dart';
import 'package:go_router/go_router.dart';

import '../../core/api_client.dart';

const _allowedExtensions = {'stl', '3mf', 'obj'};

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

  Future<void> _pickFile() async {
    final result = await FilePicker.platform.pickFiles(
      type: FileType.custom,
      allowedExtensions: _allowedExtensions.toList(),
    );
    if (result == null || result.files.isEmpty) return;
    final file = result.files.first;
    if (file.path == null) return;
    await _upload(file.path!, file.name);
  }

  Future<void> _upload(String path, String filename) async {
    final ext = filename.split('.').last.toLowerCase();
    if (!_allowedExtensions.contains(ext)) {
      setState(() => _errorMsg = 'Unsupported file type: .$ext. Use STL, 3MF, or OBJ.');
      return;
    }
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
      body: Padding(
        padding: const EdgeInsets.all(24),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            _DropZone(
              loading: _loading,
              dragging: _dragging,
              onFilePicked: _pickFile,
              onDragEntered: () => setState(() => _dragging = true),
              onDragExited: () => setState(() => _dragging = false),
              onFileDrop: (path, name) {
                setState(() => _dragging = false);
                _upload(path, name);
              },
            ),
            if (_errorMsg != null) ...[
              const SizedBox(height: 16),
              Text(_errorMsg!, style: TextStyle(color: Theme.of(context).colorScheme.error)),
            ],
            if (_meshInfo != null) ...[
              const SizedBox(height: 24),
              _MeshInfoCard(info: _meshInfo!),
              const SizedBox(height: 16),
              FilledButton.icon(
                icon: const Icon(Icons.tune),
                label: const Text('Slice Settings'),
                onPressed: () => context.go('/settings/${_meshInfo!['job_id']}'),
              ),
            ],
          ],
        ),
      ),
    );
  }
}

class _DropZone extends StatelessWidget {
  final bool loading;
  final bool dragging;
  final VoidCallback onFilePicked;
  final VoidCallback onDragEntered;
  final VoidCallback onDragExited;
  final void Function(String path, String name) onFileDrop;

  const _DropZone({
    required this.loading,
    required this.dragging,
    required this.onFilePicked,
    required this.onDragEntered,
    required this.onDragExited,
    required this.onFileDrop,
  });

  @override
  Widget build(BuildContext context) {
    final cs = Theme.of(context).colorScheme;
    return DropTarget(
      onDragEntered: (_) => onDragEntered(),
      onDragExited: (_) => onDragExited(),
      onDragDone: (details) {
        if (loading || details.files.isEmpty) return;
        final xfile = details.files.first;
        onFileDrop(xfile.path, xfile.name);
      },
      child: GestureDetector(
        onTap: loading ? null : onFilePicked,
        child: AnimatedContainer(
          duration: const Duration(milliseconds: 150),
          height: 200,
          decoration: BoxDecoration(
            color: dragging ? cs.primary.withOpacity(0.08) : null,
            border: Border.all(
              color: dragging ? cs.primary : cs.outline,
              style: BorderStyle.solid,
              width: dragging ? 2.5 : 2,
            ),
            borderRadius: BorderRadius.circular(16),
          ),
          child: Center(
            child: loading
                ? const CircularProgressIndicator()
                : Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Icon(
                        dragging ? Icons.file_download : Icons.upload_file,
                        size: 48,
                        color: dragging ? cs.primary : cs.primary,
                      ),
                      const SizedBox(height: 12),
                      Text(
                        dragging
                            ? 'Drop to upload'
                            : 'Drop STL / 3MF / OBJ here, or tap to browse',
                      ),
                    ],
                  ),
          ),
        ),
      ),
    );
  }
}

class _MeshInfoCard extends StatelessWidget {
  final Map<String, dynamic> info;
  const _MeshInfoCard({required this.info});

  @override
  Widget build(BuildContext context) {
    final dims = info['dimensions_mm'] as Map<String, dynamic>? ?? {};
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Model loaded', style: Theme.of(context).textTheme.titleMedium),
            const Divider(),
            _Row('File', info['filename']?.toString() ?? ''),
            _Row('Triangles', _fmt(info['triangle_count'])),
            _Row('Size (mm)',
                'X:${_d(dims['x'])} Y:${_d(dims['y'])} Z:${_d(dims['z'])}'),
            _Row('Volume', '${_d(info['volume_cm3'])} cm³'),
            if (info['was_repaired'] == true)
              const Padding(
                padding: EdgeInsets.only(top: 8),
                child: Row(children: [
                  Icon(Icons.build, size: 16, color: Colors.orange),
                  SizedBox(width: 4),
                  Text('Mesh was auto-repaired', style: TextStyle(color: Colors.orange)),
                ]),
              ),
          ],
        ),
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
        padding: const EdgeInsets.symmetric(vertical: 4),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Text(label, style: Theme.of(context).textTheme.bodySmall),
            Text(value),
          ],
        ),
      );
}
