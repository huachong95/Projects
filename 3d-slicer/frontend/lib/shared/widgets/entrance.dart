import 'package:flutter/material.dart';

import '../../theme/app_theme.dart';

/// Fade + upward slide entrance. Wrap any widget; pass an [index] to stagger a
/// list/grid so items cascade in instead of popping.
class FadeSlideIn extends StatefulWidget {
  final Widget child;
  final int index;
  final Duration delayPer;
  final double offsetY;

  const FadeSlideIn({
    super.key,
    required this.child,
    this.index = 0,
    this.delayPer = const Duration(milliseconds: 70),
    this.offsetY = 18,
  });

  @override
  State<FadeSlideIn> createState() => _FadeSlideInState();
}

class _FadeSlideInState extends State<FadeSlideIn>
    with SingleTickerProviderStateMixin {
  late final AnimationController _c =
      AnimationController(vsync: this, duration: AppMotion.enter);

  @override
  void initState() {
    super.initState();
    Future<void>.delayed(widget.delayPer * widget.index, () {
      if (mounted) _c.forward();
    });
  }

  @override
  void dispose() {
    _c.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final curved = CurvedAnimation(parent: _c, curve: AppMotion.curve);
    return AnimatedBuilder(
      animation: curved,
      builder: (_, child) => Opacity(
        opacity: curved.value,
        child: Transform.translate(
          offset: Offset(0, widget.offsetY * (1 - curved.value)),
          child: child,
        ),
      ),
      child: widget.child,
    );
  }
}
