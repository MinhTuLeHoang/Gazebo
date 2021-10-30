# Cách để hiện bầu trời trong gazebo:

- Mở file .world, kiếm tag ```<scence></scence>```
- Thêm vào:
```ruby
<sky>
    <clouds>
    <speed> 0 </speed>
    </clouds>
</sky>
```
- Ta được như sau:
```
<scene>
    <ambient>0.4 0.4 0.4 1</ambient>
    <background>0.7 0.7 0.7 1</background>
    <shadows>true</shadows>

    <sky>
        <clouds>
        <speed> 0 </speed>
        </clouds>
    </sky>
</scene>
```

# Các thế giới đã build xong:

- circle_world.world (dùng để test bo đường lane):
