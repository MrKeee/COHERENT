'''
Env0:室内apartment
四个room:bedroom,livingroom,children room,kitchen
四个机器人,两个机械臂,一个无人机,一个机器狗
Task:(bedroom的房门关着,childrenroom开着)

inside_<bedroom floor>(1)_<bedroom>(0)
inside_<childroom floor>(3)_<childroom>(2)
inside_<livingroom floor>(5)_<livingroom>(4)
inside_<kitchen floor>(7)_<kitchen>(6)
leading to_<door>(8)_<bedroom>(0)
leading to_<door>(8)_<livingroom>(4)
leading to_<door>(9)_<childroom>(2)
leading to_<door>(9)_<livingroom>(4)
leading to_<door>(10)_<livingroom>(4)
leading to_<door>(10)_<kitchen>(6)
on_<sofa>(11)_<livingroom floor>(5)
on_<coffee table>(12)_<livingroom floor>(5)
on_<dining table>(13)_<livingroom floor>(5)
on_<fridge>(14)_<kitchen floor>(7)
on_<microwave>(15)_<dining table>(13)
on_<coffeemachine>(16)_<dining table>(13)
on_<apple>(17)_<coffee table>(12)
inside_<banana>(18)_<fridge>(14)
inside_<cup>(19)_<fridge>(14)
on_<hat>(20)_<coffee table>(12)
inside_<bread>(21)_<fridge>(14)
on_<quadrotor>(22)_<bedroom floor>(1)
inside_<quadrotor>(22)_<bedroom>(0)
on_<robot dog>(23)_<livingroom floor>(5)
inside_<robot dog>(23)_<livingroom>(4)
on_<robot arm>(24)_<dining table>(13)
inside_<robot arm>(24)_<livingroom>(4)
with_<quadrotor>(22)_<basket>(25)
on_<basket>(25)_<bedroom floor>(1)
inside_<bread>(26)_<microwave>(15)
on_<bed>(27)_<bedroom floor>(1)
on_<bed>(28)_<childroom floor>(3)
on_<socks>(29)_<sofa>(11)
on_<milkbox>(30)_<dining table>(13)
on_<socks>(31)_<sofa>(11)
on_<kitchen table>(32)_<kitchen floor>(7)
on_<high table>(33)_<bedroom floor>(1)
on_<cabinet>(34)_<childroom floor>(3)
on_<high kitchen table>(35)_<kitchen floor>(7)
on_<baby bottle>(36)_<dining table>(13)
on_<bag of chips>(37)_<kitchen floor>(7)
on_<bag of tea>(38)_<kitchen floor>(7)
on_<battery>(39)_<high table>(33)
on_<flashlight>(40)_<kitchen table>(32)
on_<dinner napkin>(41)_<childroom floor>(3)
on_<vegetable peeler>(42)_<high kitchen table>(35)
on_<notebook>(43)_<bed>(27)
on_<textbook>(44)_<bed>(28)
on_<toy box>(45)_<childroom floor>(3)
on_<toy car>(46)_<childroom floor>(3)
on_<wallet>(47)_<bed>(27)
on_<silver coin>(48)_<high table>(33)
on_<orange>(49)_<bedroom floor>(1)
on_<franka arm>(50)_<high table>(33)
on_<plate>(51)_<dining table>(13)
on_<box>(52)_<high table>(33)
inside_<plate>(53)_<fridge>(14)
on_<knife>(54)_<coffee table>(12)
on_<sunglasses>(55)_<dining table>(13)

新加的Add object:
baby_bottle
bag_of_chips
bag_of_tea
battery
flashlight
dinner_napkin
vegetable_peeler
Notebook
Textbook
Toybox
Toy_car
wallet
Silver_coins
Orange

Single_agent:
新加的任务
0、无人机在childrenroom地板起飞,飞到kitchen 的高table降落(机器狗初始化在厨房,无人机在孩子房间)
无人机起飞,飞向living room,飞向kitchen,飞向high kitchen table,降落(5步)
The <quadrotor>(22) lands on the <high kitchen table>(35).
on_<quadrotor>(22)_<high kitchen table>(35)
1、无人机在children room地板起飞,飞到living room的diningtable降落(狗在客厅,无人机在孩子房间)
无人机起飞,飞向living room,飞向dining table,降落(4步)
The <quadrotor>(22) lands on the <dining table>(13).
on_<quadrotor>(22)_<dining table>(13)

2、机械臂把微波炉里的面包拿出来放到盘子上,桌上的boxmilk放到微波炉里面
机械臂打开微波炉,抓面包,放到盘子上,抓milkbox,放到微波炉里面(5步)
Put the <bread>(26) into the <plate>(51) which is on the <dining table>(13) and put the <milkbox>(30) into the <microwave>(15). The <bread>(26) is originally in the <microwave>(15)."
        
inside_<bread>(26)_<plate>(51)
inside_<milkbox>(30)_<microwave>(15)
on_<plate>(51)_<dining table>(13)


3、卧室里的机械臂把桌上的硬币和电池放入box
抓硬币,放入box,抓电池,放入box(4步)
Put the <silver coin>(48) and <battery>(39) into the <box>(52).
inside_<silver coin>(48)_<box>(52)
inside_<battery>(39)_<box>(52)

Single-agent task
4、冰箱里面的cup放到coffee table上,
狗走向厨房,走向冰箱,打开冰箱门,走向cup,抓起cup,走向livingroom ,走向coffee table,放下cup(8步)
Put the <cup>(19) on the <coffee table>(12). The <cup>(19) is originally in the <fridge>(14) which is in the <kitchen>(6).
on_<cup>(19)_<coffee table>(12)

5、coffee table上的apple放入冰箱里面的plate(狗)
GT:机器狗走向coffee table,抓苹果,走向kitchen,走向kitchen的矮桌子,放下苹果,走向冰箱,打开冰箱,走向苹果,抓起苹果,走向冰箱,苹果放盘子里(11步)
Put the <apple>(17) into the <plate>(53) which is in the <fridge>(14). The <plate>(53) is in the <fridge>(14) which is in the <kitchen>(6).

（5、狗在厨房把厨房桌子上的苹果放入冰箱盘子里
狗走向冰箱，开冰箱，走向苹果，抓苹果，走向冰箱，放入冰箱盘子（6步）
修改后
）


inside_<apple>(17)_<plate>(53)
inside_<plate>(53)_<fridge>(14)

6、沙发上的hat放到卧室里面的床上
狗走向门,开门,走向hat,抓起hat,走向bedroom,走向床,放下hat(7步)
Put the <hat>(20) on the <bed>(27).
on_<hat>(20)_<bed>(27)

7、床上的notebook放到coffee table上(狗)
Put the <notebook>(43) on the <coffee table>(12).
on_<notebook>(43)_<coffee table>(12)
GT:狗走向门,开门,走进bedroom,走向notebook,抓起notebook,走向living room,走向coffee table,放下(8步)

8、把sofa上的一双手套放到childrensroom的床上(狗)(无人机在孩子房,狗在客厅)
Put the <socks>(29) an <socks>(31) on the <bed>(28).
on_<socks>(29)_<bed>(28)
on_<socks>(31)_<bed>(28)
GT:狗走向sofa,抓手套1,走向children room,走向bed,放手套1,走向living room,走向sofa,抓手套2,走向children room,走向bed,放手套1(11步)
(8,改成把sofa上的一只手套放到childrensroom的床上(狗)(无人机在孩子房,狗在客厅)
狗走向sofa,抓手套,走向children room,走向bed,放手套,共5步)
Put the <socks>(29) on the <bed>(28).
on_<socks>(29)_<bed>(28)


Two-agent task
9、把餐桌上的plate(51)放到厨房的kitchen table上(机械臂与无人机)(无人机初始在living room)(狗初始在厨房)
The <quadrotor>(22) lands on the <high kitchen table>(35) with the <plate>(51) in the <basket>(25).
inside_<plate>(51)_<basket>(25)
on_<quadrotor>(22)_<high kitchen table>(35)
无人机起飞,飞向餐桌,降落,机械臂抓盘子,放无人机篮子里,无人机起飞,飞向厨房,飞向高桌子,降落(9步)

10、把冰箱里面的banana运输到kitchen table上(机器狗与无人机)(无人机初始在kitchen)
The <quadrotor>(22) lands on the <high kitchen table>(35) with the <banana>(18) in the <basket>(25). The <banana>(18) is originally in the <fridge>(14) which is in the <kitchen>(6).
inside_<banana>(18)_<basket>(25)
on_<quadrotor>(22)_<high kitchen table>(35)
GT:狗走向厨房,走向冰箱,打开冰箱,走向香蕉，抓香蕉,狗走向无人机,香蕉放入篮子,无人机起飞,飞向high kitchen table, 降落(10步)

11、将橙子从 bedroom地面上拿到厨房高桌子上(无人机+狗)(无人机初始在厨房)狗初始在bedroom
The <quadrotor>(22) lands on the <high kitchen table>(35) with the <orange>(49) in the <basket>(25). The <orange>(49) is originally in the <bedroom>(0).
inside_<orange>(49)_<basket>(25)
on_<quadrotor>(22)_<high kitchen table>(35)

狗走向门,开门,走向橙子,抓起来,走向living room,走向kitchen,狗走向无人机,橙子放入篮子,无人机起飞,飞向high kitchen table, 降落(11步)(无人机初始在厨房)

12、厨房地面上的薯片和茶叶罐运到厨房的高桌子(无人机初始在厨房,狗初始在living room)
The <quadrotor>(22) lands on the <high kitchen table>(35) with the <bag of chips>(37) and <bag of tea>(38) in the <basket>(25).
inside_<bag of chips>(37)_<basket>(25)
inside_<bag of tea>(38)_<basket>(25)
on_<quadrotor>(22)_<high kitchen table>(35)
狗走向厨房,走向薯片,抓起来,走向无人机篮子,放进去,狗走向茶叶罐,抓起来,走向无人机篮子,放进去,无人机起飞,飞向high kitchen table,降落(12步)

13、Livingroom的coffee table上的刀放到厨房的高桌子上(狗初始在kitchen,无人机初始在living room)
The <quadrotor>(22) lands on the <high kitchen table>(35) with the <knife>(54) in the <basket>(25).
inside_<knife>(54)_<basket>(25)
on_<quadrotor>(22)_<high kitchen table>(35)
机器狗走向living room,走向knife,抓起来,走向无人机,放进去,无人机起飞,飞向厨房,飞向high table,降落(9步)

Three-agent task
14、餐桌上的sunglasses放到childrenroom的cabinet里面(无人机+狗+臂)无人机初始化在children room,狗初始化在living room
Put the <sunglasses>(55) into the <cabinet>(34).
inside_<sunglasses>(55)_<cabinet>(34)
无人机起飞,飞向living room,飞向餐桌,降落,机械臂抓sunglasses,放入无人机篮子里,无人机起飞,飞向children room,降落,机器狗走向children room ,走向橱柜,开橱柜,走向无人机篮子,抓起太阳镜,走向橱柜,把太阳镜放进去。(16步)
（修改成无人机初始在living room，狗在children room
无人机起飞，飞向餐桌,降落,机械臂抓sunglasses,放入无人机篮子里,无人机起飞,飞向children room,降落,机器狗走走向橱柜,开橱柜,走向无人机篮子,抓起太阳镜,走向橱柜,把太阳镜放进去。(14步)
）

15、餐桌上milkbox放到冰箱里面(无人机+狗+臂)狗初始在厨房,无人机初始在living room
Put the <milkbox>(30) into the <fridge>(14).
inside_<milkbox>(30)_<fridge>(14)
无人机起飞,飞向餐桌,降落,机械臂抓milkbox,放入无人机篮子,无人机起飞,飞向厨房,降落,狗走向冰箱,开冰箱,走向无人机,抓牛奶,走向冰箱,放入牛奶。(14步)

16、微波炉里面的bread和coffee table上的knife放到dining table上的plate里面(无人机+狗+臂)狗和无人机初始在living room
Put the <bread>(26) and <knife>(54) into the <plate>(51) which is on the <dining table>(13). The <bread>(26) is originally in the <microwave>(15).

inside_<bread>(26)_<plate>(51)
inside_<knife>(54)_<plate>(51)
on_<plate>(51)_<dining table>(13)
狗走向刀，抓刀，走向无人机篮子，放进去，无人机起飞，飞向餐桌，降落，机械臂抓刀，放盘子里，臂开微波炉，抓面包，放盘子里(12步)


17、dining table上的奶瓶放进冰箱,冰箱里的cup放到厨房的低桌子上(狗初始在kitchen,无人机在living room)
Put the <baby bottle>(36) into the <fridge>(14) and put the <cup>(19) on the <kitchen table>(32). The <cup>(19) is originally in the <fridge>(14).
inside_<baby bottle>(36)_<fridge>(14)
on_<cup>(19)_<kitchen table>(32)
无人机起飞,飞向diningtable,降落,机械臂抓奶瓶,放无人机篮子,无人机起飞,飞向厨房,降落,狗走向冰箱,开门,走向啤酒瓶,抓,走向table,放,走向奶瓶,抓,走向冰箱,放(18步)
（
修改成dining table上的奶瓶放进冰箱(狗初始在kitchen,无人机在living room)
无人机起飞,飞向diningtable,降落,机械臂抓奶瓶,放无人机篮子,无人机起飞,飞向厨房,降落,狗走向冰箱,开门,走向奶瓶,抓,走向冰箱,放(14步)
）
Put the <baby bottle>(36) into the <fridge>(14).
inside_<baby bottle>(36)_<fridge>(14)


18、机器狗和无人机初始点在children room,把floor上的玩具车放到toy_box,然后把餐巾纸放到diningtable上
Put the <toy car>(46) into the <toy box>(45) and put the <dinner napkin>(41) on the <dining table>(13).
inside_<toy car>(46)_<toy box>(45)
on_<dinner napkin>(41)_<dining table>(13)
狗走向玩具车,抓起来,走向toy_box,放进去,走向餐巾纸,抓起来,走向无人机,放进去,无人机起飞,飞向living room,飞向dining table,降落,机械臂抓餐巾纸,放到桌子上(14步)

19、狗和无人机初始在children room,把孩子房间床上的textbook放到bedroom的高桌子上
Put the <textbook>(44) on the <high table>(33). 
on_<textbook>(44)_<high table>(33)
狗走向textbook,抓起来放,走向无人机,放进去,狗走向living room ,走向door,开门,无人机起飞,飞向living room,飞向bedroom,飞向高桌子,降落,抓textbook,放到桌子上(14步)

20、手电筒从厨房的矮桌子拿到bedroom的高桌子上的盒子里(狗初始在厨房,无人机初始在living room)
Put the <flashlight>(40) into the <box>(52).
inside_<flashlight>(40)_<box>(52)
机器狗走向手电筒,抓起来,走向living room,走向无人机,放到无人机篮子里面,走向door,开门,无人机起飞,飞向bedroom,飞向高桌子,降落,机械臂抓手电筒,放到box里面(13步)
'''