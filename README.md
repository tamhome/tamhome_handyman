# HandyMan


## Demonstration Video (YouTube)

<p align="center">
    <a href="https://youtu.be/HtXxb13liv0">
    <img src="config/Tam@Home S-OPL Handyman Demonstration.jpg" alt="HandyMan">
    </a>
</p>

## How to launch

- Terminal 1

```bash
singularity shell -B /run/user/1000,/var/lib/dbus/machine-id --nv env/sandbox_sigverse/
source /entrypoint.sh
source devel/setup.bash
roslaunch tamhome_hsrb_utils sigverse_rosbridge_server.launch
```

- Unityを起動

- Terminal 2

```bash
singularity shell -B /run/user/1000,/var/lib/dbus/machine-id --nv env/sandbox_sigverse/
source /entrypoint.sh
source devel/setup.bash
roslaunch sigverse_hsrb_utils bringup.launch
```

- Terminal 3
  - langsamのDocker化が終わるまではローカルで動作させる

```bash
source devel/setup.bash
roslaunch tam_object_detection hsr_head_rgbd_lang_sam_service.launch
```

- Terminal 4
  - Terminal 3のコマンドにて，langsamサーバを起動させてから実行する
  - 最後のコマンドに `wait_to_ready:=false` が**含まれていない**ことを確認

```bash
singularity shell -B /run/user/1000,/var/lib/dbus/machine-id --nv env/sandbox_sigverse/
source /entrypoint.sh
source devel/setup.bash
roslaunch tamhome_handyman handyman.launch
```
