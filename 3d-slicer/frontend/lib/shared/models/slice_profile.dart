class SliceProfile {
  final String name;
  // Layer
  final double layerHeight;
  final double firstLayerHeight;
  // Infill
  final int infillDensity;
  final String infillPattern;
  // Walls & shells
  final int wallCount;
  final int topLayers;
  final int bottomLayers;
  // Supports
  final bool supportsEnabled;
  final String supportType;
  // Brim
  final double brimWidth;
  // Temperature
  final double printTemp;
  final double bedTemp;
  final int fanSpeed;
  final int firstLayerFanSpeed;
  // Speed
  final double printSpeed;
  final double firstLayerSpeed;
  // Retraction
  final double retractionDistance;
  final double retractionSpeed;
  // Z-hop
  final double zHop;
  // Seam
  final String seamPosition;

  const SliceProfile({
    required this.name,
    this.layerHeight = 0.2,
    this.firstLayerHeight = 0.2,
    this.infillDensity = 15,
    this.infillPattern = 'grid',
    this.wallCount = 3,
    this.topLayers = 4,
    this.bottomLayers = 4,
    this.supportsEnabled = false,
    this.supportType = 'normal',
    this.brimWidth = 0,
    this.printTemp = 215,
    this.bedTemp = 60,
    this.fanSpeed = 100,
    this.firstLayerFanSpeed = 0,
    this.printSpeed = 60,
    this.firstLayerSpeed = 25,
    this.retractionDistance = 0.8,
    this.retractionSpeed = 45,
    this.zHop = 0,
    this.seamPosition = 'aligned',
  });

  Map<String, dynamic> toOverrides() => {
        'layer_height': layerHeight,
        'layer_height_0': firstLayerHeight,
        'infill_sparse_density': infillDensity,
        'infill_pattern': infillPattern,
        'wall_line_count': wallCount,
        'top_layers': topLayers,
        'bottom_layers': bottomLayers,
        'support_enable': supportsEnabled,
        'support_type': supportType,
        'brim_width': brimWidth,
        'material_print_temperature': printTemp,
        'material_bed_temperature': bedTemp,
        'cool_fan_speed': fanSpeed,
        'cool_fan_speed_0': firstLayerFanSpeed,
        'speed_print': printSpeed,
        'speed_layer_0': firstLayerSpeed,
        'retraction_amount': retractionDistance,
        'retraction_retract_speed': retractionSpeed,
        'retraction_hop': zHop,
        'z_seam_type': seamPosition,
      };

  SliceProfile copyWith({
    String? name,
    double? layerHeight,
    double? firstLayerHeight,
    int? infillDensity,
    String? infillPattern,
    int? wallCount,
    int? topLayers,
    int? bottomLayers,
    bool? supportsEnabled,
    String? supportType,
    double? brimWidth,
    double? printTemp,
    double? bedTemp,
    int? fanSpeed,
    int? firstLayerFanSpeed,
    double? printSpeed,
    double? firstLayerSpeed,
    double? retractionDistance,
    double? retractionSpeed,
    double? zHop,
    String? seamPosition,
  }) =>
      SliceProfile(
        name: name ?? this.name,
        layerHeight: layerHeight ?? this.layerHeight,
        firstLayerHeight: firstLayerHeight ?? this.firstLayerHeight,
        infillDensity: infillDensity ?? this.infillDensity,
        infillPattern: infillPattern ?? this.infillPattern,
        wallCount: wallCount ?? this.wallCount,
        topLayers: topLayers ?? this.topLayers,
        bottomLayers: bottomLayers ?? this.bottomLayers,
        supportsEnabled: supportsEnabled ?? this.supportsEnabled,
        supportType: supportType ?? this.supportType,
        brimWidth: brimWidth ?? this.brimWidth,
        printTemp: printTemp ?? this.printTemp,
        bedTemp: bedTemp ?? this.bedTemp,
        fanSpeed: fanSpeed ?? this.fanSpeed,
        firstLayerFanSpeed: firstLayerFanSpeed ?? this.firstLayerFanSpeed,
        printSpeed: printSpeed ?? this.printSpeed,
        firstLayerSpeed: firstLayerSpeed ?? this.firstLayerSpeed,
        retractionDistance: retractionDistance ?? this.retractionDistance,
        retractionSpeed: retractionSpeed ?? this.retractionSpeed,
        zHop: zHop ?? this.zHop,
        seamPosition: seamPosition ?? this.seamPosition,
      );
}
