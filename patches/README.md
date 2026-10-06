# Patches for the contact_graspnet submodule

`grasp-overlay-topic.diff` makes `grasp_processor` publish two images the GUI
subscribes to:

- `/grasp_overlay` (sensor_msgs/Image, bgr8): the grasp visualization, every
  inference, instead of only writing `grasps_<n>.png` when `save_plots` is
  set. The PNG behaviour is unchanged.
- `/segmentation` (sensor_msgs/Image, mono8): the UOIS label map. The
  publisher already existed upstream but nothing ever called publish on it.

It lives here as a patch because the submodule repo
(aidankirwin/contact_graspnet_ros2) is not writable by this branch's author.

Apply inside the submodule:

```bash
cd contact_graspnet
git apply ../patches/grasp-overlay-topic.diff
```

Then rebuild the cgn workspace. Aidan: if this looks right, commit it to
cgn_singleContainerTest and delete this patch dir.
