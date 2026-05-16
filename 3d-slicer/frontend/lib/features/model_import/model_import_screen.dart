import 'dart:io';

import 'package:dio/dio.dart';
import 'package:file_picker/file_picker.dart';
import 'package:flutter/material.dart';
import 'package:go_router/go_router.dart';

import '../../core/api_client.dart';

class ModelImportScreen extends StatefulWidget {
  const ModelImportScreen({super.key});

  @override
  State<ModelImportScreen> createState() => _ModelImportScreenState();
}

class _ModelImportScreenState extends State<ModelImportScreen> {
  bool _loading = false;
  String? _errorMsg;
  Map<String, dynamic>? _meshInfo;

  Future<void> _pickFile() async {
    final result = await FilePicker.platform.pickFiles(
      type: FileType.custom,
      allowedExtensions: ['stl', '3mf', 'obj'],
    );
    if (result == null || result.files.isEmpty) return;
    final file = result.files.first;
    if (file.path == null) return;
    await _upload(file.path!, file.name);
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
      body: Padding(
        padding: const EdgeInsets.all(24),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            _DropZone(
              loading: _loading,
              onFilePicked: _pickFile,
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
  final VoidCallback onFilePicked;

  const _DropZone({required this.loading, required this.onFilePicked});

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: loading ? null : onFilePicked,
      child: Container(
        height: 200,
        decoration: BoxDecoration(
          border: Border.all(
            color: Theme.of(context).colorScheme.outline,
            style: BorderStyle.solid,
            width: 2,
          ),
          borderRadius: BorderRadius.circular(16),
        ),
        child: Center(
          child: loading
              ? const CircularProgressIndicator()
              : Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    Icon(Icons.upload_file, size: 48, color: Theme.of(context).colorScheme.primary),
                    const SizedBox(height: 12),
                    const Text('Drop STL / 3MF / OBJ here, or tap to browse'),
                  ],
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
