'''
Task:(这个场景增加一个储物室,储物室里面有一个箱子和一个储物架,箱子packing_box是开的)(如果没有特殊说明,两个机械臂初始在kitchen,狗和无人机初始在living room)

inside_<livingroom floor>(1)_<livingroom>(0)
inside_<kitchen floor>(3)_<kitchen>(2)
on_<coffee table>(4)_<livingroom floor>(1)
on_<chair>(5)_<livingroom floor>(1)
on_<chair>(6)_<livingroom floor>(1)
on_<fridge>(7)_<kitchen floor>(3)
on_<kitchen table>(11)_<kitchen floor>(3)
on_<sink>(8)_<kitchen table>(11)
on_<dining table>(9)_<kitchen floor>(3)
on_<burner>(10)_<kitchen table>(11)
on_<hand towel>(12)_<kitchen table>(11)
on_<plate>(13)_<kitchen table>(11)
inside_<apple>(14)_<plate>(13)
inside_<tomato>(15)_<plate>(13)
on_<bowl>(17)_<kitchen table>(11)
inside_<salad>(16)_<bowl>(17)
on_<tomato sauce jar>(18)_<kitchen table>(11)
on_<frying pan>(19)_<burner>(10)
on_<robot arm>(20)_<kitchen table>(11)
inside_<robot arm>(20)_<kitchen>(2)
on_<robot dog>(21)_<livingroom floor>(1)
inside_<robot dog>(21)_<livingroom>(0)
on_<quadrotor>(22)_<livingroom floor>(1)
inside_<quadrotor>(22)_<livingroom>(0)
with_<quadrotor>(22)_<basket>(23)
on_<basket>(23)_<livingroom floor>(1)
leading to_<door>(24)_<livingroom>(0)
leading to_<door>(24)_<kitchen>(2)
inside_<bread>(25)_<fridge>(7)
on_<magazine>(26)_<low bookshelf>(37)
on_<remote control>(27)_<chair>(5)
on_<cookbook>(28)_<low bookshelf>(36)
on_<toy car>(29)_<livingroom floor>(1)
on_<fruit bowl>(30)_<dining table>(9)
on_<cooking pot>(31)_<kitchen floor>(3)
on_<dish rack>(32)_<sink>(8)
on_<cutlery set>(33)_<dining table>(9)
leading to_<door>(35)_<livingroom>(0)
leading to_<door>(35)_<utility room>(34)
on_<low bookshelf>(36)_<livingroom floor>(1)
on_<low bookshelf>(37)_<livingroom floor>(1)
on_<storage shelf>(38)_<utility room floor>(45)
on_<packing box>(39)_<utility room floor>(45)
on_<plate>(40)_<kitchen table>(11)
on_<cleaning supplies>(41)_<kitchen floor>(3)
on_<old clothes>(42)_<kitchen floor>(3)
inside_<pencil box>(43)_<packing box>(39)
on_<robot arm>(44)_<dining table>(9)
inside_<robot arm>(44)_<kitchen>(2)
inside_<utility room floor>(45)_<utility room>(34)
on_<empty bottle>(46)_<coffee table>(4)
on_<trash can>(47)_<kitchen floor>(3)
inside_<beer bottle>(48)_<fridge>(7)
on_<cup>(49)_<coffee table>(4)
inside_<knife>(50)_<fridge>(7)
inside_<fork>(51)_<fridge>(7)
inside_<meat ball>(52)_<fridge>(7)
on_<beer glass>(53)_<coffee table>(4)
on_<mirror>(54)_<storage shelf>(38)

Single-agent task:
新添的任务:
0、将livingroom地板上的玩具车搬运到储物室的箱子里
Put the <toy car>(29) into the <packing box>(39) which is in the <utility room>(34).
inside_<toy car>(29)_<packing box>(39)
机械狗走向门35,开门,走向玩具车,抓起来,走向储物室,走向箱子,放进去(7 步)

1、厨房地面上的旧衣服搬运到客厅的椅子上
Put the <old clothes>(42) on the <chair>(5).
on_<old clothes>(42)_<chair>(5)
狗走向厨房,走向旧衣服,抓起来,走向客厅,走向椅子,放下(6步)

2、把餐厅高桌子上的苹果和tomato拿到另一个空盘子上,将剩下的那个盘子拿到水槽里
Put the <apple>(14) and <tomato>(15) into the <plate>(40) and put the <plate>(13) into the <sink>(8).
inside_<apple>(14)_<plate>(40)
inside_<tomato>(15)_<plate>(40)
inside_<plate>(13)_<sink>(8)
机械臂抓apple,放到盘子上,抓tomato,放到盘子上,抓盘子,放到水槽里(6步)

3、机器狗到储物室,把箱子里面的工具箱拿到货架上
Put the <pencil box>(43) on the <storage shelf>(38). They are both in the <utility room>(34).
on_<pencil box>(43)_<storage shelf>(38)
狗走向门35,开门,走向储物室,走向箱子,抓起来pencilbox,走向货架,放下(7步)

4、livingroom桌面上的空瓶子放到kitchen的垃圾桶里
Put the <empty bottle>(46) into the <trash can>(47).
inside_<empty bottle>(46)_<trash can>(47)
狗走向空瓶子,抓起来,走向kitchen,走向垃圾桶,扔进去(5步)

5、冰箱里面的beer bottle放到living room的桌面上(狗)
Put the <beer bottle>(48) on the <coffee table>(4). The <beer bottle>(48) is originally in the <fridge>(7).
on_<beer bottle>(48)_<coffee table>(4)
狗走向kitchen,走向冰箱,开冰箱,走向juice,抓juice,走向livingroom,走向coffee table,juice放到桌子上(8步)

6、把储物室的一个packing_box里面d pencil box放到coffee table上(狗)
Put the <pencil box>(43) on the <coffee table>(4). The <pencil box>(43) is originally in the <utility room>(34).
on_<pencil box>(43)_<coffee table>(4)
狗走向储物室的门,开门,走向pencilbox,抓起来,走向living room,走近coffee table,把pencilbox放上去(7步)

Two-agent task:
7、把客厅的矮书架上的食谱搬运到厨房的kitchen table上
The <quadrotor>(22) lands on the <kitchen table>(11) with the <cookbook>(28) in the <basket>(23). 
inside_<cookbook>(28)_<basket>(23)
on_<quadrotor>(22)_<kitchen table>(11)
狗走向书架,拿书,走向无人机篮子,放进去,无人机起飞,飞向厨房,飞向kitchen table,降落(8步)

8、冰箱里面的面包运到diningtable上(狗初始在kitchen)
The <quadrotor>(22) lands on the <dining table>(9) with the <bread>(25) in the <basket>(23). The <bread>(25) is originally in the <fridge>(7).
inside_<bread>(25)_<basket>(23)
on_<quadrotor>(22)_<dining table>(9)
狗走向冰箱,打开冰箱,走向面包,抓起来,走向living room,走向无人机篮子,放进去,无人机起飞,飞向kitchen,飞向dining table,降落(11步)

9、dining table上的fruit bowl放到kitchentable的水槽里(无人机初始在kitchen)
Put the <fruit bowl>(30) into the <sink>(8). 
inside_<fruit bowl>(30)_<sink>(8)
无人机起飞,飞向dining table,降落,机械臂抓fruit bowl,放入篮子,无人机起飞,飞向kitchen table,降落,机械臂抓fruit bowl,放入水槽(10步)

10、Kitchentable上的plate40放到diningtable上（无人机初始在kitchen）
Put the <plate>(40) on the <dining table>(9).
on_<plate>(40)_<dining table>(9)
无人机起飞,飞向kitchen table,降落,机械臂抓plate,放入篮子,无人机起飞,飞向dining table,降落,机械臂抓plate,放下(10步)
#############
10,进行一下修改:Kitchentable上的plate40放到冰箱里面（无人机和狗都初始在kitchen）
Put the <plate>(40) into the <fridge>(7).
inside_<plate>(40)_<fridge>(7)
无人机起飞,飞向kitchen table,降落,机械臂抓plate,放入篮子,无人机起飞,飞向kitchen floor,降落,狗走向冰箱，开冰箱，走向plate，抓起来，走向冰箱，放进去(14步)
####################


11、kitchen table上的苹果和tomato用无人机送到dining table上(无人机+机械臂)无人机初始在kitchen
The <quadrotor>(22) lands on the <dining table>(9) with the <apple>(14) and <tomato>(15) in the <basket>(23). 
inside_<apple>(14)_<basket>(23)
inside_<tomato>(15)_<basket>(23)
on_<quadrotor>(22)_<dining table>(9)
无人机起飞,飞向kitchen table,降落,机械臂抓苹果,放进篮子,抓tomato,放进篮子,无人机起飞,飞向dining table,降落(10步)

12、把livingroom中的两个书架上的杂志和食谱运送到kitchen中diningtable上(狗+无人机)
The <quadrotor>(22) lands on the <dining table>(9) with the <cookbook>(28) and <magazine>(26) in the <basket>(23). 
inside_<cookbook>(28)_<basket>(23)
inside_<magazine>(26)_<basket>(23)
on_<quadrotor>(22)_<dining table>(9)
狗走向书架1,拿杂志,走向篮子,放进篮子,走向书架2,拿食谱,走向篮子,放进篮子,无人机起飞,飞向厨房,飞向dining table,降落(12步)

13、厨房位置冰箱里面的叉子和刀运输到diningtable上(狗+无人机)(无人机和狗的初始位置在厨房)
The <quadrotor>(22) lands on the <dining table>(9) with the <knife>(50) and <fork>(51) in the <basket>(23). The <knife>(50) and <fork>(51) are originally in the <fridge>(7).
inside_<knife>(50)_<basket>(23)
inside_<fork>(51)_<basket>(23)
on_<quadrotor>(22)_<dining table>(9)
狗走向冰箱,打开冰箱,狗走向叉子,抓,走向无人机篮子,放进去,走向刀,抓起来,走向篮子,放进去,无人机起飞,飞向dining table,降落(13步)

Three-agent task:
14、厨房冰箱里面的meatball放到kitchen table炉灶的平底锅里面(无人机+狗+臂)
Put the <meat ball>(52) into the <frying pan>(19) which is on the <burner>(10). The <meat ball>(52) is originally in the <fridge>(7).
inside_<meat ball>(52)_<frying pan>(19)
on_<frying pan>(19)_<burner>(10)
狗走进厨房,走向冰箱,打开,走向肉丸,抓起肉丸,走向living room,走向无人机,放入篮子,无人机起飞,飞向kitchen,飞向kitchen table ,降落,机械臂抓鸡翅,放入平底锅(14步)

15、Livingroom的桌子上的glass和cup放到厨房的sink里面(无人机+狗+臂)
Put the <beer glass>(53) and <cup>(49) into the <sink>(8).
inside_<beer glass>(53)_<sink>(8)
inside_<cup>(49)_<sink>(8)
狗走向glass,抓起来,走向篮子,放入,走向plate,抓起来,走向无人机篮子,放入,无人机起飞,飞向kitchen,飞向kitchen table,降落,机械臂抓plate,放入水池,抓glass,放入水池(16步)

16、厨房桌台上的毛巾放到livingroom上的coffeetable上(无人机+狗+臂)
Put the <hand towel>(12) on the <coffee table>(4).
on_<hand towel>(12)_<coffee table>(4)
无人机起飞,飞向kitchen,飞向kitchen table ,降落,机械臂抓毛巾,放入篮子,无人机起飞,飞向living room,降落,机器狗走向篮子,抓毛巾,走向coffee table,放下(13步)

17、将厨房餐桌上的apple放到livingroom桌子上(无人机+狗+臂)
Put the <apple>(14) on the <coffee table>(4).
on_<apple>(14)_<coffee table>(4)
无人机起飞,飞向kitchen,飞向kitchen table ,降落,机械臂抓apple,放入篮子,无人机起飞,飞向living room,降落,机器狗走向篮子,抓apple,走向coffee table,放下(13步)

18、将厨房地板上的清洁剂放到水池里在
Put the <cleaning supplies>(41) into the <sink>(8).
inside_<cleaning supplies>(41)_<sink>(8)
狗走向厨房,走向清洁剂,抓起来,走向living room,走向无人机篮子,放入,无人机起飞,飞向kitchen,飞向kitchen table,降落,机械臂抓清洁剂,放入水池(12步)

19、将厨房地板上的锅放到炉灶上
Put the <cooking pot>(31) on the <burner>(10).
on_<cooking pot>(31)_<burner>(10)
狗走向厨房,走向锅,抓起来,走向living room,走向无人机篮子,放入,无人机起飞,飞向kitchen,飞向kitchen table,降落,机械臂抓锅,到炉灶上(12步)

20、无人机初始在储藏室货架上的镜子放到dining table的餐桌上(无人机初始在储藏室)
Put the <mirror>(54) on the <dining table>(9).
on_<mirror>(54)_<dining table>(9)
狗走向门35,开门,走向储物室,走向镜子,抓起来，走向无人机篮子,放入,无人机起飞,飞向living room，飞向kitchen,飞向dining table,降落,机械臂抓镜子,放桌子上(14步)

'''



# 环境整理顺序
# 1、先删注释（已完成，2024/1/22）
# 2、检查一下环境的整体描述relation_<class name>(id)_<class name>(id)是否正确 （已完成，2024/1/22）
# 2、检查env.py文件里面task的object的id是否正确(已检查21个task的<class name>(id)是与整体环境一致，已完成，2024/1/22)
# 3、检查object的属性和状态(已完成，2024/1/22)
# 4、检查任务的GT（检查了task1，3，7，8，11，10，16，20的GT并加了上去，无误，已完成，2024/1/22）
# 5、把task的谓语逻辑与instruction加上去(八个任务检查无误，已完成，2024/1/22，其他任务待检查)
# 6、检查task中的初始位置是否正确（八个任务检查无误，已完成，，2024/1/22，其他任务待检查）
# 7、对task进行删减，包括agent的初始位置的更改(八个任务删减完毕，并且用脚本验证过了，每一个在例表中的子物体，他的父物体也必须在列表中，已完成，2024/1/22，其他任务待检查)