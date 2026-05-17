import 'package:flutter/material.dart';

import '../../core/api_client.dart';

class NotificationSettingsScreen extends StatefulWidget {
  const NotificationSettingsScreen({super.key});

  @override
  State<NotificationSettingsScreen> createState() => _NotificationSettingsScreenState();
}

class _NotificationSettingsScreenState extends State<NotificationSettingsScreen> {
  List<Map<String, dynamic>> _channels = [];
  List<String> _events = [];
  bool _loading = true;

  @override
  void initState() {
    super.initState();
    _load();
  }

  Future<void> _load() async {
    setState(() => _loading = true);
    try {
      final results = await Future.wait([
        apiClient.get<Map<String, dynamic>>('/api/notifications'),
        apiClient.get<Map<String, dynamic>>('/api/notifications/events'),
      ]);
      final channels =
          (results[0].data?['channels'] as List?)?.cast<Map<String, dynamic>>() ?? [];
      final events = (results[1].data?['events'] as List?)?.cast<String>() ?? [];
      setState(() {
        _channels = channels;
        _events = events;
      });
    } catch (_) {
    } finally {
      setState(() => _loading = false);
    }
  }

  Future<void> _delete(String id) async {
    try {
      await apiClient.delete('/api/notifications/$id');
      setState(() => _channels.removeWhere((c) => c['channel_id'] == id));
    } catch (e) {
      _snack('Delete failed: $e');
    }
  }

  Future<void> _test(String id) async {
    _snack('Sending test…');
    try {
      await apiClient.post('/api/notifications/$id/test');
      _snack('Test sent!');
    } catch (e) {
      _snack('Test failed: $e');
    }
  }

  Future<void> _showAddEdit([Map<String, dynamic>? existing]) async {
    final result = await showDialog<bool>(
      context: context,
      builder: (_) => _ChannelDialog(existing: existing, allEvents: _events),
    );
    if (result == true) _load();
  }

  void _snack(String msg) {
    if (mounted) ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(msg)));
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Notifications'),
        actions: [
          IconButton(icon: const Icon(Icons.refresh), onPressed: _load),
        ],
      ),
      floatingActionButton: FloatingActionButton.extended(
        onPressed: () => _showAddEdit(),
        icon: const Icon(Icons.add),
        label: const Text('Add channel'),
      ),
      body: _loading
          ? const Center(child: CircularProgressIndicator())
          : Column(
              children: [
                _EventsInfoBanner(events: _events),
                Expanded(
                  child: _channels.isEmpty
                      ? Center(
                          child: Column(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: [
                              Icon(Icons.notifications_off_outlined,
                                  size: 64, color: Colors.grey[600]),
                              const SizedBox(height: 16),
                              const Text('No channels configured'),
                              const SizedBox(height: 8),
                              const Text(
                                'Add Telegram, Discord, or webhook channels\nto get print alerts.',
                                textAlign: TextAlign.center,
                                style: TextStyle(color: Colors.grey),
                              ),
                            ],
                          ),
                        )
                      : ListView.builder(
                          padding: const EdgeInsets.fromLTRB(12, 8, 12, 80),
                          itemCount: _channels.length,
                          itemBuilder: (_, i) => _ChannelCard(
                            channel: _channels[i],
                            onEdit: () => _showAddEdit(_channels[i]),
                            onDelete: () => _delete(_channels[i]['channel_id'] as String),
                            onTest: () => _test(_channels[i]['channel_id'] as String),
                          ),
                        ),
                ),
              ],
            ),
    );
  }
}

class _EventsInfoBanner extends StatelessWidget {
  final List<String> events;
  const _EventsInfoBanner({required this.events});

  @override
  Widget build(BuildContext context) {
    if (events.isEmpty) return const SizedBox.shrink();
    return Container(
      color: Theme.of(context).colorScheme.surfaceContainerHighest,
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      child: Row(
        children: [
          const Icon(Icons.info_outline, size: 16),
          const SizedBox(width: 8),
          Expanded(
            child: Text(
              'Supported events: ${events.join(', ')}',
              style: Theme.of(context).textTheme.bodySmall,
            ),
          ),
        ],
      ),
    );
  }
}

class _ChannelCard extends StatelessWidget {
  final Map<String, dynamic> channel;
  final VoidCallback onEdit;
  final VoidCallback onDelete;
  final VoidCallback onTest;

  const _ChannelCard({
    required this.channel,
    required this.onEdit,
    required this.onDelete,
    required this.onTest,
  });

  @override
  Widget build(BuildContext context) {
    final type = channel['type'] as String? ?? 'webhook';
    final enabled = channel['enabled'] as bool? ?? true;
    final events = (channel['events'] as List?)?.cast<String>() ?? [];
    final webhookUrl = channel['webhook_url'] as String? ?? '';
    final chatId = channel['chat_id'] as String? ?? '';
    final hasToken = channel['has_token'] as bool? ?? false;

    final typeIcon = switch (type) {
      'telegram' => Icons.telegram,
      'discord' => Icons.discord,
      _ => Icons.webhook,
    };

    final typeColor = switch (type) {
      'telegram' => const Color(0xFF229ED9),
      'discord' => const Color(0xFF5865F2),
      _ => Colors.teal,
    };

    String subtitle;
    if (type == 'telegram') {
      subtitle = hasToken ? 'Chat: ${chatId.isNotEmpty ? chatId : "—"}' : 'No token set';
    } else if (type == 'discord') {
      subtitle = webhookUrl.isNotEmpty ? webhookUrl : 'No webhook URL';
    } else {
      subtitle = webhookUrl.isNotEmpty ? webhookUrl : 'No webhook URL';
    }

    return Card(
      margin: const EdgeInsets.only(bottom: 8),
      child: ListTile(
        leading: CircleAvatar(
          backgroundColor: typeColor.withOpacity(0.15),
          child: Icon(typeIcon, color: typeColor, size: 22),
        ),
        title: Row(
          children: [
            Text(type[0].toUpperCase() + type.substring(1)),
            const SizedBox(width: 8),
            if (!enabled)
              Container(
                padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                decoration: BoxDecoration(
                  color: Colors.orange.withOpacity(0.2),
                  borderRadius: BorderRadius.circular(4),
                ),
                child: const Text('disabled',
                    style: TextStyle(fontSize: 10, color: Colors.orange)),
              ),
          ],
        ),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(subtitle, style: const TextStyle(fontSize: 12)),
            const SizedBox(height: 2),
            Text(
              events.isEmpty ? 'No events' : events.join(', '),
              style: TextStyle(fontSize: 11, color: Colors.grey[600]),
              maxLines: 1,
              overflow: TextOverflow.ellipsis,
            ),
          ],
        ),
        isThreeLine: true,
        trailing: PopupMenuButton<String>(
          onSelected: (v) {
            if (v == 'edit') onEdit();
            if (v == 'test') onTest();
            if (v == 'delete') onDelete();
          },
          itemBuilder: (_) => const [
            PopupMenuItem(value: 'edit', child: Text('Edit')),
            PopupMenuItem(value: 'test', child: Text('Send test')),
            PopupMenuItem(value: 'delete', child: Text('Delete')),
          ],
        ),
      ),
    );
  }
}

class _ChannelDialog extends StatefulWidget {
  final Map<String, dynamic>? existing;
  final List<String> allEvents;
  const _ChannelDialog({this.existing, required this.allEvents});

  @override
  State<_ChannelDialog> createState() => _ChannelDialogState();
}

class _ChannelDialogState extends State<_ChannelDialog> {
  final _form = GlobalKey<FormState>();
  String _type = 'telegram';
  bool _enabled = true;
  List<String> _selectedEvents = [];
  final _botToken = TextEditingController();
  final _chatId = TextEditingController();
  final _webhookUrl = TextEditingController();
  bool _saving = false;

  @override
  void initState() {
    super.initState();
    final e = widget.existing;
    if (e != null) {
      _type = e['type'] as String? ?? 'telegram';
      _enabled = e['enabled'] as bool? ?? true;
      _selectedEvents = (e['events'] as List?)?.cast<String>().toList() ?? [];
      _webhookUrl.text = e['webhook_url'] as String? ?? '';
      _chatId.text = e['chat_id'] as String? ?? '';
    } else {
      _selectedEvents = List.from(widget.allEvents);
    }
  }

  @override
  void dispose() {
    _botToken.dispose();
    _chatId.dispose();
    _webhookUrl.dispose();
    super.dispose();
  }

  Future<void> _save() async {
    if (!_form.currentState!.validate()) return;
    setState(() => _saving = true);
    final body = {
      'type': _type,
      'enabled': _enabled,
      'events': _selectedEvents,
      'bot_token': _botToken.text.trim(),
      'chat_id': _chatId.text.trim(),
      'webhook_url': _webhookUrl.text.trim(),
    };
    try {
      final existing = widget.existing;
      if (existing != null) {
        await apiClient.patch('/api/notifications/${existing['channel_id']}', data: body);
      } else {
        await apiClient.post('/api/notifications', data: body);
      }
      if (mounted) Navigator.pop(context, true);
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(context)
            .showSnackBar(SnackBar(content: Text('Save failed: $e')));
      }
    } finally {
      if (mounted) setState(() => _saving = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    final isEdit = widget.existing != null;
    return AlertDialog(
      title: Text(isEdit ? 'Edit channel' : 'Add channel'),
      content: SizedBox(
        width: 380,
        child: Form(
          key: _form,
          child: SingleChildScrollView(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                DropdownButtonFormField<String>(
                  value: _type,
                  decoration: const InputDecoration(
                    labelText: 'Channel type',
                    border: OutlineInputBorder(),
                    isDense: true,
                  ),
                  items: const [
                    DropdownMenuItem(value: 'telegram', child: Text('Telegram')),
                    DropdownMenuItem(value: 'discord', child: Text('Discord')),
                    DropdownMenuItem(value: 'webhook', child: Text('Webhook')),
                  ],
                  onChanged: (v) => setState(() => _type = v!),
                ),
                const SizedBox(height: 12),
                if (_type == 'telegram') ...[
                  TextFormField(
                    controller: _botToken,
                    decoration: const InputDecoration(
                      labelText: 'Bot token',
                      hintText: '123456:ABC-...',
                      border: OutlineInputBorder(),
                      isDense: true,
                    ),
                    validator: (v) =>
                        (v == null || v.trim().isEmpty) ? 'Required' : null,
                  ),
                  const SizedBox(height: 12),
                  TextFormField(
                    controller: _chatId,
                    decoration: const InputDecoration(
                      labelText: 'Chat ID',
                      hintText: '-100123456',
                      border: OutlineInputBorder(),
                      isDense: true,
                    ),
                    validator: (v) =>
                        (v == null || v.trim().isEmpty) ? 'Required' : null,
                  ),
                ] else ...[
                  TextFormField(
                    controller: _webhookUrl,
                    decoration: InputDecoration(
                      labelText: _type == 'discord'
                          ? 'Discord webhook URL'
                          : 'Webhook URL',
                      border: const OutlineInputBorder(),
                      isDense: true,
                    ),
                    validator: (v) =>
                        (v == null || v.trim().isEmpty) ? 'Required' : null,
                  ),
                ],
                const SizedBox(height: 16),
                Text('Events', style: Theme.of(context).textTheme.labelLarge),
                const SizedBox(height: 6),
                Wrap(
                  spacing: 6,
                  runSpacing: 4,
                  children: widget.allEvents.map((e) {
                    final selected = _selectedEvents.contains(e);
                    return FilterChip(
                      label: Text(e, style: const TextStyle(fontSize: 11)),
                      selected: selected,
                      onSelected: (v) {
                        setState(() {
                          if (v) {
                            _selectedEvents.add(e);
                          } else {
                            _selectedEvents.remove(e);
                          }
                        });
                      },
                    );
                  }).toList(),
                ),
                const SizedBox(height: 12),
                SwitchListTile(
                  value: _enabled,
                  onChanged: (v) => setState(() => _enabled = v),
                  title: const Text('Enabled'),
                  dense: true,
                  contentPadding: EdgeInsets.zero,
                ),
              ],
            ),
          ),
        ),
      ),
      actions: [
        TextButton(
          onPressed: _saving ? null : () => Navigator.pop(context, false),
          child: const Text('Cancel'),
        ),
        FilledButton(
          onPressed: _saving ? null : _save,
          child: _saving
              ? const SizedBox(
                  width: 16,
                  height: 16,
                  child: CircularProgressIndicator(strokeWidth: 2),
                )
              : Text(isEdit ? 'Save' : 'Add'),
        ),
      ],
    );
  }
}
