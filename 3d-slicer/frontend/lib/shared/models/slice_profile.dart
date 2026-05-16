class SliceProfile {
  final String name;
  final double layerHeight;
  final int infillDensity;
  final String infillPattern;
  final bool supportsEnabled;
  final String supportType;
  final double printTemp;
  final double bedTemp;
  final double printSpeed;

  const SliceProfile({
    required this.name,
    this.layerHeight = 0.2,
    this.infillDensity = 15,
    this.infillPattern = 'grid',
    this.supportsEnabled = false,
    this.supportType = 'normal',
    this.printTemp = 215,
    this.bedTemp = 60,
    this.printSpeed = 60,
  });

  Map<String, dynamic> toOverrides() => {
        'layer_height': layerHeight,
        'infill_sparse_density': infillDensity,
        'infill_pattern': infillPattern,
        'support_enable': supportsEnabled,
        'support_type': supportType,
        'material_print_temperature': printTemp,
        'material_bed_temperature': bedTemp,
        'speed_print': printSpeed,
      };

  SliceProfile copyWith({
    String? name,
    double? layerHeight,
    int? infillDensity,
    String? infillPattern,
    bool? supportsEnabled,
    String? supportType,
    double? printTemp,
    double? bedTemp,
    double? printSpeed,
  }) =>
      SliceProfile(
        name: name ?? this.name,
        layerHeight: layerHeight ?? this.layerHeight,
        infillDensity: infillDensity ?? this.infillDensity,
        infillPattern: infillPattern ?? this.infillPattern,
        supportsEnabled: supportsEnabled ?? this.supportsEnabled,
        supportType: supportType ?? this.supportType,
        printTemp: printTemp ?? this.printTemp,
        bedTemp: bedTemp ?? this.bedTemp,
        printSpeed: printSpeed ?? this.printSpeed,
      );
}
