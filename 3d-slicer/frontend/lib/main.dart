import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import 'core/api_client.dart';
import 'router.dart';
import 'theme/app_theme.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await apiClient.init();
  runApp(const ProviderScope(child: SlicerApp()));
}

class SlicerApp extends StatelessWidget {
  const SlicerApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp.router(
      title: '3D Slicer',
      debugShowCheckedModeBanner: false,
      theme: AppTheme.dark,
      routerConfig: appRouter,
    );
  }
}
