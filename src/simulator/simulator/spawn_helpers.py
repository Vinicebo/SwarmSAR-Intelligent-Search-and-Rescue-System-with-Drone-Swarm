import os
import tempfile


def _render(template_path, **kwargs):
    with open(template_path, 'r') as f:
        content = f.read()
    return content.format(**kwargs)


def render_drone_sdf(sdf_template_path, drone_id):
    """Fill in a per-drone copy of the quadrotor SDF (sensor topics are
    templated with {drone_id} so multiple spawned drones don't publish on
    the same Gazebo topic) and write it to a temp file for spawning."""
    rendered = _render(sdf_template_path, drone_id=drone_id)
    fd, path = tempfile.mkstemp(prefix=f'{drone_id}_', suffix='.sdf')
    with os.fdopen(fd, 'w') as f:
        f.write(rendered)
    return path


def render_bridge_yaml(bridge_template_path, drone_ids, world_bridge_template_path, world_name):
    """Concatenate one rendering of the per-drone bridge template for every
    drone_id, plus one rendering of the shared world-level bridge (clock,
    force), into a single parameter_bridge config file. world_name must
    match the <world name="..."> in the loaded .world file, since both the
    force topic and /clock are scoped to it."""
    entries = [
        _render(bridge_template_path, drone_id=drone_id)
        for drone_id in drone_ids
    ]
    entries.append(_render(world_bridge_template_path, world=world_name))

    fd, path = tempfile.mkstemp(prefix='ros_gz_bridge_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        f.write('\n'.join(entries))
    return path
