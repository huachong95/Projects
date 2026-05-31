import 'dart:math' as math;

import 'package:flutter/material.dart';

import '../../theme/app_theme.dart';

/// A slow, living gradient backdrop. Two soft colour "blobs" drift behind the
/// content so screens feel alive instead of flat black. Cheap to render
/// (a single CustomPaint with a repeating animation).
class AnimatedBackground extends StatefulWidget {
  final Widget child;
  const AnimatedBackground({super.key, required this.child});

  @override
  State<AnimatedBackground> createState() => _AnimatedBackgroundState();
}

class _AnimatedBackgroundState extends State<AnimatedBackground>
    with SingleTickerProviderStateMixin {
  late final AnimationController _c =
      AnimationController(vsync: this, duration: const Duration(seconds: 18))
        ..repeat();

  @override
  void dispose() {
    _c.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        Positioned.fill(
          child: AnimatedBuilder(
            animation: _c,
            builder: (_, __) => CustomPaint(painter: _BlobPainter(_c.value)),
          ),
        ),
        widget.child,
      ],
    );
  }
}

class _BlobPainter extends CustomPainter {
  final double t;
  _BlobPainter(this.t);

  @override
  void paint(Canvas canvas, Size size) {
    canvas.drawRect(Offset.zero & size, Paint()..color = AppColors.bg);

    final angle = t * 2 * math.pi;
    _blob(
      canvas,
      size,
      Offset(size.width * (0.25 + 0.10 * math.sin(angle)),
          size.height * (0.18 + 0.06 * math.cos(angle))),
      size.shortestSide * 0.55,
      AppColors.primary.withOpacity(0.16),
    );
    _blob(
      canvas,
      size,
      Offset(size.width * (0.82 + 0.08 * math.cos(angle * 0.8)),
          size.height * (0.78 + 0.05 * math.sin(angle * 1.2))),
      size.shortestSide * 0.5,
      AppColors.cool.withOpacity(0.10),
    );
  }

  void _blob(Canvas canvas, Size size, Offset center, double radius, Color color) {
    final paint = Paint()
      ..shader = RadialGradient(
        colors: [color, color.withOpacity(0)],
      ).createShader(Rect.fromCircle(center: center, radius: radius));
    canvas.drawCircle(center, radius, paint);
  }

  @override
  bool shouldRepaint(covariant _BlobPainter old) => old.t != t;
}
