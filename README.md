# o3d_1-12_bull_3_npc1 #

Looking at some simple steering behaviours, this one is the dynamic **Arrive** pattern, this is key to implementing path following. 

### Keys ###

|Key|Movement|Method|
|---|---|---|
| w | Forward  | ```forward()``` |
| a | Right  | ```turnRight()``` |
| j | Jump  | ```jump()``` |
| f | Fly  | ```fly()``` |