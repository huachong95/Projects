import 'package:flutter/material.dart';

import '../../core/api_client.dart';
import '../../shared/widgets/animated_background.dart';
import '../../shared/widgets/entrance.dart';
import '../../shared/widgets/hover_scale.dart';
import '../../shared/widgets/shimmer.dart';
import '../../theme/app_theme.dart';

class TimelapseScreen extends StatefulWidget {
  const TimelapseScreen({super.key});

  @override
  State<TimelapseScreen> createState() => _TimelapseScreenState();
}

class _TimelapseScreenState extends State<TimelapseScreen> {
  List<Map<String, dynamic>> _jobs = [];
  bool _loading = true;
  String? _compilingId;

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
      if (mounted) setState(() => _jobs = jobs);
    } catch (_) {
    } finally {
      if (mounted) setState(() => _loading = false);
    }
  }

  Future<void> _compile(String jobId) async {
    setState(() => _compilingId = jobId);
    try {
      await apiClient.post('/api/timelapse/$jobId/compile');
      await _loadJobs();
      _snack('Timelapse compiled successfully');
    } catch (e) {
      _snack('Compile failed: $e');
    } finally {
      if (mounted) setState(() => _compilingId = null);
    }
  }

  void _snack(String m) {
    if (mounted) ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(m)));
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Timelapses'),
        actions: [
          IconButton(icon: const Icon(Icons.refresh), onPressed: _loadJobs),
          const SizedBox(width: 4),
        ],
      ),
      body: AnimatedBackground(
        child: SafeArea(
          top: false,
          child: _loading
              ? _buildSkeleton()
              : _jobs.isEmpty
                  ? const _EmptyState()
                  : _buildGrid(),
        ),
      ),
    );
  }

  Widget _buildSkeleton() {
    return GridView.builder(
      padding: const EdgeInsets.all(16),
      gridDelegate: _gridDelegate,
      itemCount: 6,
      itemBuilder: (_, __) => Container(
        decoration: BoxDecoration(
          color: AppColors.surface,
          borderRadius: BorderRadius.circular(AppRadius.md),
          border: Border.all(color: AppColors.surfaceBorder),
        ),
        clipBehavior: Clip.antiAlias,
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            const Expanded(child: Shimmer(child: ColoredBox(color: Colors.white))),
            Padding(
              padding: const EdgeInsets.all(12),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Shimmer.box(width: 80, height: 14),
                  const SizedBox(height: 8),
                  Shimmer.box(width: 120, height: 10),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildGrid() {
    return GridView.builder(
      padding: const EdgeInsets.all(16),
      gridDelegate: _gridDelegate,
      itemCount: _jobs.length,
      itemBuilder: (_, i) => FadeSlideIn(
        index: i,
        child: _TimelapseCard(
          job: _jobs[i],
          compiling: _compilingId == _jobs[i]['job_id'],
          onCompile: () => _compile(_jobs[i]['job_id'] as String),
        ),
      ),
    );
  }

  static const _gridDelegate = SliverGridDelegateWithMaxCrossAxisExtent(
    maxCrossAxisExtent: 300,
    crossAxisSpacing: 16,
    mainAxisSpacing: 16,
    childAspectRatio: 0.82,
  );
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
                child: const Icon(Icons.movie_outlined, size: 48, color: AppColors.onSurfaceDim),
              ),
              const SizedBox(height: 20),
              Text('No timelapses yet', style: Theme.of(context).textTheme.titleMedium),
              const SizedBox(height: 8),
              const Text(
                'Timelapses are recorded automatically during\nprints with the Prusa Camera.',
                textAlign: TextAlign.center,
                style: TextStyle(color: AppColors.onSurfaceDim, height: 1.4),
              ),
            ],
          ),
        ),
      );
}

class _TimelapseCard extends StatelessWidget {
  final Map<String, dynamic> job;
  final bool compiling;
  final VoidCallback onCompile;

  const _TimelapseCard({required this.job, required this.compiling, required this.onCompile});

  @override
  Widget build(BuildContext context) {
    final hasVideo = job['has_video'] as bool? ?? false;
    final frameCount = job['frame_count'] as int? ?? 0;
    final sizeMb = job['video_size_mb'] as double? ?? 0.0;
    final jobId = job['job_id'] as String? ?? '';
    final shortId = jobId.length >= 8 ? jobId.substring(0, 8) : jobId;

    return HoverScale(
      scale: 1.025,
      child: Container(
        decoration: BoxDecoration(
          color: AppColors.surface,
          borderRadius: BorderRadius.circular(AppRadius.md),
          border: Border.all(color: AppColors.surfaceBorder),
        ),
        clipBehavior: Clip.antiAlias,
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Expanded(
              child: Container(
                decoration: const BoxDecoration(
                  gradient: LinearGradient(
                    colors: [Color(0xFF1A1D26), Color(0xFF0C0D11)],
                    begin: Alignment.topLeft,
                    end: Alignment.bottomRight,
                  ),
                ),
                child: Center(
                  child: Icon(
                    hasVideo ? Icons.play_circle_fill : Icons.image_outlined,
                    size: 52,
                    color: hasVideo ? AppColors.primary : AppColors.onSurfaceDim,
                  ),
                ),
              ),
            ),
            Padding(
              padding: const EdgeInsets.all(12),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(shortId, style: Theme.of(context).textTheme.labelLarge),
                  const SizedBox(height: 2),
                  Text(
                    '$frameCount frames${hasVideo ? ' · ${sizeMb.toStringAsFixed(1)} MB' : ''}',
                    style: const TextStyle(color: AppColors.onSurfaceDim, fontSize: 12.5),
                  ),
                  if (!hasVideo && frameCount > 0) ...[
                    const SizedBox(height: 10),
                    SizedBox(
                      width: double.infinity,
                      child: OutlinedButton(
                        onPressed: compiling ? null : onCompile,
                        child: compiling
                            ? const SizedBox(
                                height: 16,
                                width: 16,
                                child: CircularProgressIndicator(strokeWidth: 2))
                            : const Text('Compile video'),
                      ),
                    ),
                  ],
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
