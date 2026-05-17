import 'package:flutter/material.dart';

import '../../core/api_client.dart';

class TimelapseScreen extends StatefulWidget {
  const TimelapseScreen({super.key});

  @override
  State<TimelapseScreen> createState() => _TimelapseScreenState();
}

class _TimelapseScreenState extends State<TimelapseScreen> {
  List<Map<String, dynamic>> _jobs = [];
  bool _loading = true;

  @override
  void initState() {
    super.initState();
    _loadJobs();
  }

  Future<void> _loadJobs() async {
    setState(() => _loading = true);
    try {
      final resp = await apiClient.get<Map<String, dynamic>>('/api/timelapse');
      final jobs = (resp.data?['jobs'] as List?)?.cast<Map<String, dynamic>>() ?? [];
      setState(() => _jobs = jobs);
    } catch (_) {
    } finally {
      setState(() => _loading = false);
    }
  }

  Future<void> _compile(String jobId) async {
    try {
      await apiClient.post('/api/timelapse/$jobId/compile');
      await _loadJobs();
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('Timelapse compiled successfully')),
        );
      }
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('Compile failed: $e')),
        );
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Timelapses'),
        actions: [
          IconButton(icon: const Icon(Icons.refresh), onPressed: _loadJobs),
        ],
      ),
      body: _loading
          ? const Center(child: CircularProgressIndicator())
          : _jobs.isEmpty
              ? const Center(
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Icon(Icons.videocam_outlined, size: 64, color: Colors.grey),
                      SizedBox(height: 16),
                      Text('No timelapses yet'),
                      SizedBox(height: 8),
                      Text(
                        'Timelapses are recorded automatically\nduring prints with the Prusa Camera.',
                        textAlign: TextAlign.center,
                        style: TextStyle(color: Colors.grey),
                      ),
                    ],
                  ),
                )
              : GridView.builder(
                  padding: const EdgeInsets.all(16),
                  gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(
                    maxCrossAxisExtent: 300,
                    crossAxisSpacing: 16,
                    mainAxisSpacing: 16,
                    childAspectRatio: 0.85,
                  ),
                  itemCount: _jobs.length,
                  itemBuilder: (_, i) => _TimelapseCard(
                    job: _jobs[i],
                    onCompile: () => _compile(_jobs[i]['job_id'] as String),
                  ),
                ),
    );
  }
}

class _TimelapseCard extends StatelessWidget {
  final Map<String, dynamic> job;
  final VoidCallback onCompile;

  const _TimelapseCard({required this.job, required this.onCompile});

  @override
  Widget build(BuildContext context) {
    final hasVideo = job['has_video'] as bool? ?? false;
    final frameCount = job['frame_count'] as int? ?? 0;
    final sizeMb = (job['video_size_mb'] as num?)?.toDouble() ?? 0.0;
    final jobId = job['job_id'] as String? ?? '';

    return Card(
      clipBehavior: Clip.antiAlias,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          Expanded(
            child: Container(
              color: Colors.black87,
              child: Center(
                child: hasVideo
                    ? Icon(Icons.play_circle_outline, size: 48,
                        color: Theme.of(context).colorScheme.primary)
                    : const Icon(Icons.image_outlined, size: 48, color: Colors.grey),
              ),
            ),
          ),
          Padding(
            padding: const EdgeInsets.all(12),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  jobId.substring(0, 8),
                  style: Theme.of(context).textTheme.labelLarge,
                ),
                Text(
                  '$frameCount frames${hasVideo ? ' · ${sizeMb.toStringAsFixed(1)} MB' : ''}',
                  style: Theme.of(context).textTheme.bodySmall,
                ),
                const SizedBox(height: 8),
                if (!hasVideo && frameCount > 0)
                  SizedBox(
                    width: double.infinity,
                    child: OutlinedButton(
                      onPressed: onCompile,
                      child: const Text('Compile video'),
                    ),
                  ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}
