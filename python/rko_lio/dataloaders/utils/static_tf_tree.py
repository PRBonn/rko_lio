# MIT License
#
# Copyright (c) 2025 Meher V.R. Malladi.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import numpy as np
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm


def create_static_tf_tree(bag):
    """
    Build a static TF tree from only '/tf_static' messages.

    Returns:
        dict: { child_frame_id: (parent_frame_id, transform) }
    """
    tf_tree = {}
    if not "/tf_static" in bag.topics:
        return tf_tree

    tf_static_connections = [
        conn for conn in bag.connections if (conn.topic == "/tf_static")
    ]
    for _, _, rawdata in tqdm(
        bag.messages(connections=tf_static_connections),
        desc="/tf_static messages",
        total=bag.topics["/tf_static"].msgcount,
    ):
        msg = bag.deserialize(rawdata, "tf2_msgs/msg/TFMessage")

        for transform_stamped in msg.transforms:
            child = transform_stamped.child_frame_id
            parent = transform_stamped.header.frame_id

            t = transform_stamped.transform.translation
            q = transform_stamped.transform.rotation
            T = np.eye(4)
            T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            T[:3, 3] = [t.x, t.y, t.z]

            tf_tree[child] = (parent, T)

    return tf_tree


def query_static_tf(tf_tree, from_frame, to_frame):
    """
    Compute transform matrix from 'from_frame' to 'to_frame' using a static TF tree.

    Raises ValueError if frames are disconnected.
    """
    available_frames = set(tf_tree.keys()) | {parent for parent, _ in tf_tree.values()}
    missing = [f for f in (from_frame, to_frame) if f not in available_frames]
    if missing:
        raise ValueError(
            f"Frame(s) {missing} not found in static TF tree. "
            f"Available frames: {sorted(available_frames)}. You can specify frames with CLI options."
        )
    # Walk up from "from_frame" to root, collecting transforms
    from_chain = []
    cur = from_frame
    while cur in tf_tree:
        parent, T = tf_tree[cur]
        from_chain.append((cur, parent, T))
        cur = parent
    root_from = cur
    # need this in case its the immediate ancestor
    from_chain.append((root_from, None, np.eye(4)))

    # Walk up from "to_frame" to root
    to_chain = []
    cur = to_frame
    while cur in tf_tree:
        parent, T = tf_tree[cur]
        to_chain.append((cur, parent, T))
        cur = parent
    root_to = cur
    # need this in case its the immediate ancestor
    to_chain.append((root_to, None, np.eye(4)))

    # If roots differ, the tree is disconnected because we missed some dynamic tfs in between
    if root_from != root_to:
        raise ValueError(f"No static TF path between '{from_frame}' and '{to_frame}'.")

    # Map child -> index for from_chain for quick ancestor lookup
    from_index = {f: i for i, (f, _, _) in enumerate(from_chain)}

    # Find common ancestor
    common_ancestor = None
    to_common_idx = None
    for i, (f, _, _) in enumerate(to_chain):
        if f in from_index:
            common_ancestor = f
            to_common_idx = i
            break

    # Compose transforms: from_frame -> ancestor
    T_from_to_ancestor = np.eye(4)
    for _, _, T in from_chain[: from_index[common_ancestor]]:
        T_from_to_ancestor = T @ T_from_to_ancestor

    # Compose transforms: to_frame -> ancestor
    T_to_to_ancestor = np.eye(4)
    for _, _, T in to_chain[:to_common_idx]:
        T_to_to_ancestor = T @ T_to_to_ancestor

    # Invert "to_frame -> ancestor" to get ancestor -> to_frame
    T_ancestor_to_to = np.linalg.inv(T_to_to_ancestor)

    # from_frame -> to_frame
    return T_ancestor_to_to @ T_from_to_ancestor
