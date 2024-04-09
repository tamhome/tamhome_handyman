# cartographerを用いてmapを作成する(.pbstreamから.yaml .pbm)
###### mapを作成する際にcartographerを用いて作成する場合少し特殊な方法でmapを保存します。
## mapを保存する方法
```
rosservice call /write_state
```
## pbstreamをyaml pbmに変更する
```
rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=name.pbstream -map_filestem=newname
```

