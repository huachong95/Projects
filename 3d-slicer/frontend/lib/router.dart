import 'package:go_router/go_router.dart';

import 'features/home/home_screen.dart';
import 'features/model_import/model_import_screen.dart';
import 'features/viewer/viewer_screen.dart';
import 'features/slice_settings/slice_settings_screen.dart';
import 'features/print_monitor/print_monitor_screen.dart';
import 'features/timelapse/timelapse_screen.dart';

final appRouter = GoRouter(
  initialLocation: '/',
  routes: [
    GoRoute(path: '/', builder: (_, __) => const HomeScreen()),
    GoRoute(path: '/import', builder: (_, __) => const ModelImportScreen()),
    GoRoute(
      path: '/viewer/:jobId',
      builder: (_, state) => ViewerScreen(jobId: state.pathParameters['jobId']!),
    ),
    GoRoute(
      path: '/settings/:jobId',
      builder: (_, state) => SliceSettingsScreen(jobId: state.pathParameters['jobId']!),
    ),
    GoRoute(path: '/monitor', builder: (_, __) => const PrintMonitorScreen()),
    GoRoute(path: '/timelapse', builder: (_, __) => const TimelapseScreen()),
  ],
);
