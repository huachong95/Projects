import 'package:flutter/material.dart';

import '../../theme/app_theme.dart';

/// Wraps a tappable surface so it lifts, scales and glows on hover/press —
/// the kind of micro-interaction that makes desktop UIs feel responsive.
class HoverScale extends StatefulWidget {
  final Widget child;
  final VoidCallback? onTap;
  final double scale;
  final Color glow;
  final BorderRadius? borderRadius;

  const HoverScale({
    super.key,
    required this.child,
    this.onTap,
    this.scale = 1.03,
    this.glow = AppColors.primary,
    this.borderRadius,
  });

  @override
  State<HoverScale> createState() => _HoverScaleState();
}

class _HoverScaleState extends State<HoverScale> {
  bool _hover = false;
  bool _down = false;

  @override
  Widget build(BuildContext context) {
    final radius = widget.borderRadius ?? BorderRadius.circular(AppRadius.md);
    final scale = _down ? 0.98 : (_hover ? widget.scale : 1.0);
    return MouseRegion(
      cursor: widget.onTap != null ? SystemMouseCursors.click : MouseCursor.defer,
      onEnter: (_) => setState(() => _hover = true),
      onExit: (_) => setState(() => _hover = false),
      child: GestureDetector(
        onTapDown: (_) => setState(() => _down = true),
        onTapUp: (_) => setState(() => _down = false),
        onTapCancel: () => setState(() => _down = false),
        onTap: widget.onTap,
        child: AnimatedScale(
          scale: scale,
          duration: AppMotion.fast,
          curve: AppMotion.curve,
          child: AnimatedContainer(
            duration: AppMotion.fast,
            decoration: BoxDecoration(
              borderRadius: radius,
              boxShadow: _hover
                  ? [
                      BoxShadow(
                        color: widget.glow.withOpacity(0.28),
                        blurRadius: 24,
                        spreadRadius: -4,
                        offset: const Offset(0, 8),
                      ),
                    ]
                  : const [],
            ),
            child: widget.child,
          ),
        ),
      ),
    );
  }
}
