import os
uid = os.getuid()
rows = []
total_instances = total_watches = 0
for pid in os.listdir('/proc'):
    if not pid.isdigit():
        continue
    proc = f'/proc/{pid}'
    try:
        if os.stat(proc).st_uid != uid:
            continue
        with open(f'{proc}/comm', errors='replace') as f:
            comm = f.read().strip()
        instances = watches = 0
        for fd in os.listdir(f'{proc}/fd'):
            try:
                if os.readlink(f'{proc}/fd/{fd}') != 'anon_inode:inotify':
                    continue
                instances += 1
                try:
                    with open(f'{proc}/fdinfo/{fd}', errors='replace') as info:
                        watches += sum(line.startswith('inotify ') for line in info)
                except OSError:
                    pass
            except OSError:
                pass
        if instances or watches:
            rows.append((watches, instances, int(pid), comm))
            total_instances += instances
            total_watches += watches
    except (OSError, ValueError):
        pass
with open('/proc/sys/fs/inotify/max_user_watches') as f:
    watch_limit = int(f.read())
with open('/proc/sys/fs/inotify/max_user_instances') as f:
    instance_limit = int(f.read())
print(f'watches: {total_watches}/{watch_limit} ({total_watches/watch_limit:.2%}), remaining={watch_limit-total_watches}')
print(f'instances: {total_instances}/{instance_limit} ({total_instances/instance_limit:.2%}), remaining={instance_limit-total_instances}')
print('WATCHES  INSTANCES  PID      COMMAND')
for watches, instances, pid, comm in sorted(rows, reverse=True)[:15]:
    print(f'{watches:7d}  {instances:9d}  {pid:7d}  {comm}')