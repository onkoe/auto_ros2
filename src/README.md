# `src/`: The ROS 2 Nodes

This folder contains all of our ROS 2 nodes.

## Adding a New Node

To add a new node, you'll need to pick either Python or Rust for the project. Copy the (TODO: some python template) or (TODO: rust template) folder.

### Rust

- TODO.

### Python

- Add your new package the root `/pyproject.toml` under the `[tool.uv.workspace] members` section.
- Rename the node's `pyproject.toml` (`src/<new_python_node_name>/pyproject.toml`) to the node's new name.
- Do the same in the node's `package.xml`.
