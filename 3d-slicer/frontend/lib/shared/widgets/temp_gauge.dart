import 'dart:math' as math;

import 'package:flutter/material.dart';

import '../../theme/app_theme.dart';

/// Animated circular temperature gauge. The arc sweeps smoothly to the current
/// temperature (relative to target/max) and the number counts up/down.
class TempGauge extends StatelessWidget {
  final String label;
  final double current;
  final double target;
  final Color color;
  final double max;
  final IconData icon;

  const TempGauge({
    super.key,
    required this.label,
    required this.current,
    required this.target,
    required this.color,
    required this.icon,
    this.max = 300,
  });

  @override
  Widget build(BuildContext context) {
    final heating = target > 0 && (target - current).abs() > 2;
    return TweenAnimationBuilder<double>(
      tween: Tween(begin: 0, end: current.clamp(0, max)),
      duration: AppMotion.slow,
      curve: AppMotion.curve,
      builder: (context, value, _) {
        return Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            SizedBox(
              width: 116,
              height: 116,
              child: CustomPaint(
                painter: _GaugePainter(
                  fraction: (value / max).clamp(0.0, 1.0),
                  targetFraction: (target / max).clamp(0.0, 1.0),
                  color: color,
                ),
                child: Center(
                  child: Column(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(icon, size: 18, color: color),
                      const SizedBox(height: 2),
                      Text(
                        '${value.round()}°',
                        style: const TextStyle(
                          fontSize: 26,
                          fontWeight: FontWeight.w800,
                          letterSpacing: -1,
                        ),
                      ),
                      Text(
                        target > 0 ? '→ ${target.round()}°' : 'off',
                        style: TextStyle(
                          fontSize: 11,
                          color: heating ? color : AppColors.onSurfaceDim,
                          fontWeight: FontWeight.w600,
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ),
            const SizedBox(height: 8),
            Text(
              label,
              style: const TextStyle(
                color: AppColors.onSurfaceDim,
                fontWeight: FontWeight.w600,
                fontSize: 12,
                letterSpacing: 0.4,
              ),
            ),
          ],
        );
      },
    );
  }
}

class _GaugePainter extends CustomPainter {
  final double fraction;
  final double targetFraction;
  final Color color;

  _GaugePainter({
    required this.fraction,
    required this.targetFraction,
    required this.color,
  });

  static const _start = math.pi * 0.75; // 135°
  static const _sweep = math.pi * 1.5; // 270°

  @override
  void paint(Canvas canvas, Size size) {
    final rect = Offset.zero & size;
    final center = rect.center;
    final radius = size.width / 2 - 8;
    final arcRect = Rect.fromCircle(center: center, radius: radius);

    final track = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 9
      ..strokeCap = StrokeCap.round
      ..color = AppColors.surfaceHigh;
    canvas.drawArc(arcRect, _start, _sweep, false, track);

    // Target tick.
    if (targetFraction > 0) {
      final tickAngle = _start + _sweep * targetFraction;
      final p1 = center +
          Offset(math.cos(tickAngle), math.sin(tickAngle)) * (radius - 9);
      final p2 = center +
          Offset(math.cos(tickAngle), math.sin(tickAngle)) * (radius + 5);
      canvas.drawLine(
        p1,
        p2,
        Paint()
          ..color = color.withOpacity(0.55)
          ..strokeWidth = 2
          ..strokeCap = StrokeCap.round,
      );
    }

    // Value arc with gradient + glow.
    final progress = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 9
      ..strokeCap = StrokeCap.round
      ..shader = SweepGradient(
        startAngle: _start,
        endAngle: _start + _sweep,
        colors: [color.withOpacity(0.5), color],
        transform: GradientRotation(_start),
      ).createShader(arcRect)
      ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 0.6);
    canvas.drawArc(arcRect, _start, _sweep * fraction, false, progress);
  }

  @override
  bool shouldRepaint(covariant _GaugePainter old) =>
      old.fraction != fraction ||
      old.targetFraction != targetFraction ||
      old.color != color;
}
