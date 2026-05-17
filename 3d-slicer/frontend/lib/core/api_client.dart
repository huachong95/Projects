import 'package:dio/dio.dart';
import 'package:shared_preferences/shared_preferences.dart';

class ApiClient {
  static const _defaultBaseUrl = 'http://localhost:8000';
  static const _prefKey = 'backend_url';

  late final Dio _dio;
  String _baseUrl = _defaultBaseUrl;

  ApiClient() {
    _dio = Dio(BaseOptions(
      connectTimeout: const Duration(seconds: 10),
      receiveTimeout: const Duration(seconds: 30),
    ));
    _dio.interceptors.add(LogInterceptor(responseBody: false));
  }

  Future<void> init() async {
    final prefs = await SharedPreferences.getInstance();
    _baseUrl = prefs.getString(_prefKey) ?? _defaultBaseUrl;
    _dio.options.baseUrl = _baseUrl;
  }

  Future<void> setBaseUrl(String url) async {
    _baseUrl = url;
    _dio.options.baseUrl = url;
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString(_prefKey, url);
  }

  String get baseUrl => _baseUrl;

  Future<Response<T>> get<T>(String path, {Map<String, dynamic>? params}) =>
      _dio.get(path, queryParameters: params);

  Future<Response<List<int>>> getBytes(String path) =>
      _dio.get(path, options: Options(responseType: ResponseType.bytes));

  Future<void> downloadFile(String apiPath, String savePath) =>
      _dio.download(apiPath, savePath);

  Future<Response<T>> post<T>(String path, {dynamic data}) =>
      _dio.post(path, data: data);

  Future<Response<T>> put<T>(String path, {dynamic data}) =>
      _dio.put(path, data: data);

  Future<Response<T>> patch<T>(String path, {dynamic data}) =>
      _dio.patch(path, data: data);

  Future<Response<T>> delete<T>(String path) => _dio.delete(path);

  Future<Response<T>> uploadFile<T>(
    String path,
    String filePath,
    String fieldName, {
    String? filename,
  }) async {
    final formData = FormData.fromMap({
      fieldName: await MultipartFile.fromFile(filePath, filename: filename),
    });
    return _dio.post(path, data: formData);
  }
}

final apiClient = ApiClient();
