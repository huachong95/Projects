import 'package:flutter/material.dart';
import 'package:go_router/go_router.dart';

import '../../core/api_client.dart';
import '../../shared/widgets/animated_background.dart';
import '../../shared/widgets/entrance.dart';
import '../../shared/widgets/hover_scale.dart';
import '../../shared/widgets/pulsing_dot.dart';
import '../../theme/app_theme.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  bool _backendOnline = false;
  bool _checked = false;

  @override
  void initState() {
    super.initState();
    _checkBackend();
  }

  Future<void> _checkBackend() async {
    try {
      final resp = await apiClient.get<Map>('/health');
      if (mounted) setState(() {
        _backendOnline = resp.statusCode == 200;
        _checked = true;
      });
    } catch (_) {
      if (mounted) setState(() {
        _backendOnline = false;
        _checked = true;
      });
    }
  }

  static const _actions = [
    _Action(Icons.print_outlined, 'Connect Printer', 'PrusaLink / MK4 setup',
        '/connect', AppColors.primary),
    _Action(Icons.view_in_ar_outlined, 'Import Model', 'Load STL, 3MF or OBJ',
        '/import', AppColors.cool),
    _Action(Icons.monitor_heart_outlined, 'Print Monitor', 'Live status + camera',
        '/monitor', AppColors.success),
    _Action(Icons.movie_outlined, 'Timelapses', 'Watch completed prints',
        '/timelapse', AppColors.accent),
    _Action(Icons.history, 'Print History', 'Past jobs & stats',
        '/history', AppColors.cool),
    _Action(Icons.bubble_chart_outlined, 'Filament', 'Spool inventory & usage',
        '/filament', AppColors.primaryBright),
  ];

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: AnimatedBackground(
        child: SafeArea(
          child: SingleChildScrollView(
            padding: const EdgeInsets.fromLTRB(28, 28, 28, 32),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                FadeSlideIn(child: _Header(online: _backendOnline, checked: _checked)),
                const SizedBox(height: AppSpace.lg),
                if (_checked && !_backendOnline)
                  FadeSlideIn(index: 1, child: _BackendWarning(onRefresh: _checkBackend)),
                const SizedBox(height: AppSpace.xl),
                const FadeSlideIn(
                  index: 1,
                  child: Text('Quick actions',
                      style: TextStyle(
                          fontSize: 15,
                          fontWeight: FontWeight.w600,
                          color: AppColors.onSurfaceDim,
                          letterSpacing: 0.4)),
                ),
                const SizedBox(height: AppSpace.md),
                LayoutBuilder(builder: (context, c) {
                  final cols = c.maxWidth > 720 ? 2 : 1;
                  return Wrap(
                    spacing: AppSpace.md,
                    runSpacing: AppSpace.md,
                    children: [
                      for (var i = 0; i < _actions.length; i++)
                        SizedBox(
                          width: cols == 2
                              ? (c.maxWidth - AppSpace.md) / 2
                              : c.maxWidth,
                          child: FadeSlideIn(
                            index: i + 2,
                            child: _ActionCard(action: _actions[i]),
                          ),
                        ),
                    ],
                  );
                }),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

class _Header extends StatelessWidget {
  final bool online;
  final bool checked;
  const _Header({required this.online, required this.checked});

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Container(
          width: 52,
          height: 52,
          decoration: BoxDecoration(
            borderRadius: BorderRadius.circular(AppRadius.md),
            gradient: const LinearGradient(
              colors: [AppColors.primary, AppColors.primaryBright],
              begin: Alignment.topLeft,
              end: Alignment.bottomRight,
            ),
            boxShadow: [
              BoxShadow(
                color: AppColors.primary.withOpacity(0.4),
                blurRadius: 18,
                offset: const Offset(0, 6),
              ),
            ],
          ),
          child: const Icon(Icons.deblur, color: Colors.white, size: 28),
        ),
        const SizedBox(width: AppSpace.md),
        Expanded(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text('3D Slicer',
                  style: Theme.of(context).textTheme.headlineSmall),
              const SizedBox(height: 2),
              Text('Prusa MK4 workspace',
                  style: TextStyle(
                      color: AppColors.onSurfaceDim,
                      fontWeight: FontWeight.w500)),
            ],
          ),
        ),
        _StatusPill(online: online, checked: checked),
      ],
    );
  }
}

class _StatusPill extends StatelessWidget {
  final bool online;
  final bool checked;
  const _StatusPill({required this.online, required this.checked});

  @override
  Widget build(BuildContext context) {
    final color = !checked
        ? AppColors.onSurfaceDim
        : (online ? AppColors.success : AppColors.danger);
    final label = !checked ? 'Checking' : (online ? 'Backend online' : 'Backend offline');
    return AnimatedContainer(
      duration: AppMotion.med,
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 7),
      decoration: BoxDecoration(
        color: color.withOpacity(0.12),
        borderRadius: BorderRadius.circular(AppRadius.pill),
        border: Border.all(color: color.withOpacity(0.4)),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          PulsingDot(color: color, size: 8, active: online),
          const SizedBox(width: 2),
          Text(label,
              style: TextStyle(color: color, fontSize: 12.5, fontWeight: FontWeight.w600)),
        ],
      ),
    );
  }
}

class _Action {
  final IconData icon;
  final String label;
  final String description;
  final String route;
  final Color color;
  const _Action(this.icon, this.label, this.description, this.route, this.color);
}

class _ActionCard extends StatelessWidget {
  final _Action action;
  const _ActionCard({required this.action});

  @override
  Widget build(BuildContext context) {
    return HoverScale(
      glow: action.color,
      onTap: () => context.push(action.route),
      child: Container(
        padding: const EdgeInsets.all(AppSpace.lg),
        decoration: BoxDecoration(
          color: AppColors.surface,
          borderRadius: BorderRadius.circular(AppRadius.md),
          border: Border.all(color: AppColors.surfaceBorder),
        ),
        child: Row(
          children: [
            Container(
              width: 48,
              height: 48,
              decoration: BoxDecoration(
                color: action.color.withOpacity(0.14),
                borderRadius: BorderRadius.circular(AppRadius.sm),
              ),
              child: Icon(action.icon, color: action.color, size: 24),
            ),
            const SizedBox(width: AppSpace.md),
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(action.label,
                      style: Theme.of(context).textTheme.titleMedium),
                  const SizedBox(height: 3),
                  Text(action.description,
                      style: const TextStyle(
                          color: AppColors.onSurfaceDim, fontSize: 13)),
                ],
              ),
            ),
            const Icon(Icons.arrow_forward_ios,
                size: 14, color: AppColors.onSurfaceDim),
          ],
        ),
      ),
    );
  }
}

class _BackendWarning extends StatelessWidget {
  final VoidCallback onRefresh;
  const _BackendWarning({required this.onRefresh});

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(AppSpace.md),
      decoration: BoxDecoration(
        color: AppColors.danger.withOpacity(0.12),
        borderRadius: BorderRadius.circular(AppRadius.md),
        border: Border.all(color: AppColors.danger.withOpacity(0.4)),
      ),
      child: Row(
        children: [
          const Icon(Icons.warning_amber_rounded, color: AppColors.danger),
          const SizedBox(width: AppSpace.sm),
          const Expanded(
            child: Text('Backend offline. Start the Python server to continue.'),
          ),
          TextButton(onPressed: onRefresh, child: const Text('Retry')),
        ],
      ),
    );
  }
}
