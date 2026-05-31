import 'package:flutter/material.dart';

/// A status dot with a soft, continuously pulsing halo — a live "heartbeat"
/// indicator for connection / printing state.
class PulsingDot extends StatefulWidget {
  final Color color;
  final double size;
  final bool active;

  const PulsingDot({
    super.key,
    required this.color,
    this.size = 10,
    this.active = true,
  });

  @override
  State<PulsingDot> createState() => _PulsingDotState();
}

class _PulsingDotState extends State<PulsingDot>
    with SingleTickerProviderStateMixin {
  late final AnimationController _c = AnimationController(
    vsync: this,
    duration: const Duration(milliseconds: 1400),
  );

  @override
  void initState() {
    super.initState();
    if (widget.active) _c.repeat();
  }

  @override
  void didUpdateWidget(covariant PulsingDot old) {
    super.didUpdateWidget(old);
    if (widget.active && !_c.isAnimating) {
      _c.repeat();
    } else if (!widget.active && _c.isAnimating) {
      _c.stop();
    }
  }

  @override
  void dispose() {
    _c.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: _c,
      builder: (_, __) {
        final v = widget.active ? _c.value : 0.0;
        return SizedBox(
          width: widget.size * 2.6,
          height: widget.size * 2.6,
          child: Center(
            child: Stack(
              alignment: Alignment.center,
              children: [
                if (widget.active)
                  Container(
                    width: widget.size + (widget.size * 1.8 * v),
                    height: widget.size + (widget.size * 1.8 * v),
                    decoration: BoxDecoration(
                      shape: BoxShape.circle,
                      color: widget.color.withOpacity((1 - v) * 0.45),
                    ),
                  ),
                Container(
                  width: widget.size,
                  height: widget.size,
                  decoration: BoxDecoration(
                    shape: BoxShape.circle,
                    color: widget.color,
                    boxShadow: [
                      BoxShadow(
                        color: widget.color.withOpacity(0.6),
                        blurRadius: 8,
                      ),
                    ],
                  ),
                ),
              ],
            ),
          ),
        );
      },
    );
  }
}
