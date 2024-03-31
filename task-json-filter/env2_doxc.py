'''

Env3:杂货店
只有一个房间
栅格的桌子和收银台上各放一个机械臂,无人机和机器狗是在地板上

inside_<grocery store floor>(1)_<grocery store>(0)
on_<display table>(5)_<grocery store floor>(1)
on_<display bin>(2)_<display table>(5)
on_<display bin>(3)_<display table>(5)
on_<display bin>(4)_<display table>(5)
on_<pedestal table>(6)_<grocery store floor>(1)
on_<pedestal table>(7)_<grocery store floor>(1)
on_<checkout counter>(8)_<grocery store floor>(1)
inside_<apple>(9)_<display bin>(2)
inside_<apple>(10)_<display bin>(2)
inside_<banana>(11)_<display bin>(2)
inside_<banana>(12)_<display bin>(2)
inside_<lettuce>(13)_<display bin>(3)
inside_<lettuce>(14)_<display bin>(3)
inside_<carton of milk>(15)_<display bin>(4)
inside_<bottle of apple juice>(16)_<display bin>(4)
on_<bottle of orange juice>(17)_<pedestal table>(6)
on_<robot arm>(18)_<display table>(5)
inside_<robot arm>(18)_<grocery store>(0)
on_<robot arm>(19)_<checkout counter>(8)
inside_<robot arm>(19)_<grocery store>(0)
on_<robot dog>(20)_<grocery store floor>(1)
inside_<robot dog>(20)_<grocery store>(0)
on_<quadrotor>(21)_<grocery store floor>(1)
inside_<quadrotor>(21)_<grocery store>(0)
with_<quadrotor>(21)_<basket>(22)
on_<basket>(22)_<grocery store floor>(1)
on_<trash can>(23)_<grocery store floor>(1)
on_<cash register>(24)_<checkout counter>(8)
on_<paper money>(25)_<pedestal table>(7)
on_<carton>(26)_<grocery store floor>(1)
inside_<pack of bread>(27)_<carton>(26)
inside_<toilet tissue>(28)_<carton>(26)
on_<grocery shelf>(29)_<grocery store floor>(1)
on_<shopping basket>(30)_<checkout counter>(8)
on_<box of candy>(31)_<grocery shelf>(29)
on_<box of candy>(32)_<grocery shelf>(29)
on_<wallet>(33)_<grocery store floor>(1)
on_<paper money>(34)_<grocery store floor>(1)
on_<chocolate bar>(35)_<checkout counter>(8)
on_<paper money>(36)_<checkout counter>(8)
on_<receipt>(37)_<checkout counter>(8)
on_<trash can>(38)_<grocery store floor>(1)
on_<empty box>(39)_<pedestal table>(6)
on_<rag>(40)_<pedestal table>(7)
on_<bucket>(41)_<grocery store floor>(1)
on_<display table>(42)_<grocery store floor>(1)
on_<coin>(43)_<checkout counter>(8)
inside_<apple>(44)_<carton>(26)
inside_<biscuits>(45)_<shopping basket>(30)
inside_<biscuits>(46)_<shopping basket>(30)

Single-agent task:
0、把硬纸盒carton.n.02_1的货物pack_of_bread.n.01_1,toilet_tissue.n.01_1都放到货架shelf上(狗)
Put the <pack of bread>(27) and <toilet tissue>(28) on the <grocery shelf>(29).
on_<pack of bread>(27)_<grocery shelf>(29)
on_<toilet tissue>(28)_<grocery shelf>(29)
GT:狗走向硬纸盒,抓pack_of_bread.n.01_1,走向货架,放物体,走向硬纸盒,抓toilet_tissue.n.01_1,走向货架,放物体(8步)

1、把货架上的两盒糖果分别放到两个圆桌子上
Put the <box of candy>(31) on the <pedestal table>(6) and put the <box of candy>(32) on the <pedestal table>(7).
on_<box of candy>(31)_<pedestal table>(6)
on_<box of candy>(32)_<pedestal table>(7)
狗走向货架,抓box_of_candy.n.01_1,走向桌子a,放物体,走向货架,抓box_of_candy.n.01_2,走向桌子b,放物体(8步)

2、把地面上的钱包和papermoney放到桌子上
Put the <wallet>(33) and <paper money>(34) on the <pedestal table>(7).
on_<wallet>(33)_<pedestal table>(7)
on_<paper money>(34)_<pedestal table>(7)
狗走向钱包,抓钱包,走向桌子,放钱包,走向paper money,抓papermoney,走向桌子,放papermoney(8步)

3、收银台上机械臂把桌上的巧克力棒,paper money,receipt放到购物篮中
Put the <chocolate bar>(35), <paper money>(36), and <receipt>(37) into the <shopping basket>(30).
inside_<chocolate bar>(35)_<shopping basket>(30)
inside_<paper money>(36)_<shopping basket>(30)
inside_<receipt>(37)_<shopping basket>(30)
机械臂抓巧克力棒,放入篮子,抓paper money,放入篮子,抓receipt,放入篮子(6步)

##把桌上的money放入收银机里面
4、收银台上的机械臂把购物袋中的商品1,商品2放入无人机篮子里(无人机停在收银台上，这是个高桌子)
Put the <biscuits>(45) and <biscuits>(46) into the <basket>(22).
inside_<biscuits>(45)_<basket>(22)
inside_<biscuits>(46)_<basket>(22)
on_<quadrotor>(21)_<checkout counter>(8)
机械臂抓商品1,放入篮子,抓商品2,放入篮子(4步)


5、把圆桌1上的空box丢到垃圾桶里,圆桌2上的抹布丢到水桶里面(狗)
Put the <empty box>(39) into the <trash can>(38) and put the <rag>(40) into the <bucket>(41).
inside_<empty box>(39)_<trash can>(38)
inside_<rag>(40)_<bucket>(41)
GT:狗走向圆桌1,抓空box,走向垃圾桶,扔进去,走向圆桌2,抓抹布,走向bucket,扔进去(8步)

Two-agent task:
6、shelf上的两盒candy运输到display2上
The <quadrotor>(21) lands on the <display table>(42) with the <box of candy>(31) and <box of candy>(32) in the <basket>(22).
inside_<box of candy>(31)_<basket>(22)
inside_<box of candy>(32)_<basket>(22)
on_<quadrotor>(21)_<display table>(42)
狗走向货架,抓box_of_candy.n.01_1,走向无人机篮子,放进去,狗走向货架,抓box_of_candy.n.01_2,走向篮子,放入篮子,无人机起飞,飞向display2,降落(11步)

7、display1上抓apple放到收银台的basket里面
Put the <apple>(9) into the <shopping basket>(30). 
inside_<apple>(9)_<shopping basket>(30)
无人机起飞,飞向display1,降落,机械臂抓apple,放入篮子,无人机起飞,飞向收银台,降落,机械臂抓apple,放入篮子(10步)

8、狗从桌子1和桌子2上抓orange juice和paper money到display2上
The <quadrotor>(21) lands on the <display table>(42) with the <bottle of orange juice>(17) and <paper money>(25) in the <basket>(22).
inside_<bottle of orange juice>(17)_<basket>(22)
inside_<paper money>(25)_<basket>(22)
on_<quadrotor>(21)_<display table>(42)
狗走向桌子1,抓orange juice,走向无人机篮子,放进去,狗走向桌子2,抓paper money,走向篮子,放进去,无人机起飞,飞向display2,降落(11步)

9、无人机初始在display1上,里面有饼干盒和一盒果汁,放到shelf货架上(无人机在高桌子上)
无人机起飞，飞向floor，降落，机器狗走向篮子，抓biscuits，走向货架，放进去，机器狗走向篮子，抓bottle_of_apple_juice，走向货架，放进去，无人机起飞，飞向display1，降落(11步)
Put the <biscuits>(45) and <bottle of apple juice>(16) on the <grocery shelf>(29).
on_<biscuits>(45)_<grocery shelf>(29)
on_<bottle of apple juice>(16)_<grocery shelf>(29)


10、把shelf上的一盒candy,和硬纸盒里的bread运输到display2上(无人机+狗)
The <quadrotor>(21) lands on the <display table>(42) with the <box of candy>(31) and <pack of bread>(27) in the <basket>(22). 
inside_<box of candy>(31)_<basket>(22)
inside_<pack of bread>(27)_<basket>(22)
on_<quadrotor>(21)_<display table>(42)
GT:狗走向shelf,抓物品1,走向篮子,放进去,走向硬纸盒,抓物品2,走向篮子,放进去,无人机起飞,飞向display2 ,降落(11步)

11、把收银台上的巧克力棒放回到栅格4里(机械臂+无人机)
Put the <chocolate bar>(35) into the <display bin>(4).
inside_<chocolate bar>(35)_<display bin>(4)
GT:无人机起飞,飞向收银台,降落,机械臂抓巧克力棒,放入篮子里,无人机起飞,飞向dislpay1,降落,机械臂抓巧克力棒,放入栅格(10步)
####################
task11进行一下修改：把收银台上的巧克力棒放到<pedestal table>(6)上(机械臂+无人机+狗)
Put the <chocolate bar>(35) on the <pedestal table>(6).
on_<chocolate bar>(35)_<pedestal table>(6)
GT:无人机起飞,飞向收银台,降落,机械臂抓巧克力棒,放入篮子里,无人机起飞,飞向floor,降落,狗走向巧克力棒,抓起来，走向pedestal table，放在桌子上(12步)
##########

12、把栅格桌子上的banana运输到收银台桌子上的basket里面,桌面上面硬币也放到basket里(机械臂1+机械臂2+无人机)
Put the <banana>(11) and  <coin>(43) inside the <shopping basket>(30).
inside_<banana>(11)_<shopping basket>(30)
inside_<coin>(43)_<shopping basket>(30)
GT:无人机起飞,飞向display ,降落,机械臂抓banana,放入无人机篮子,无人机起飞,飞向吧台,降落,机械臂抓banana,放入basket,机械臂抓coin,放入basket(12步)

Three-agent task:
13、货架上取商品一盒candy放到收银台的basket里面,然后从收银台上的小票放到圆桌1上
Put the <box of candy>(31) into the <shopping basket>(30) and put the <receipt>(37) on the <pedestal table>(6).
inside_<box of candy>(31)_<shopping basket>(30)
on_<receipt>(37)_<pedestal table>(6)
GT:狗走向货架,抓box_of_candy.n.01_1,走向basket,放进去,无人机起飞,飞向收银台,降落,机械臂抓box_of_candy.n.01_1,放入篮子,机械臂抓receipt,放入篮子,无人机起飞,飞向floor,降落,机器狗走向篮子,抓box_of_candy.n.01_1,走向桌子,放到桌子上(18步)
(修改为货架上取商品一盒candy放到收银台的basket里面,然后从收银台上的小票放到display table1的display bin<2>上
GT:狗走向货架,抓box_of_candy.n.01_1,走向basket,放进去,无人机起飞,飞向收银台,降落,机械臂抓box_of_candy.n.01_1,放入篮子,机械臂抓receipt,放入篮子,无人机起飞,飞向display table,降落,机械臂抓box_of_candy.n.01_1,放到桌子上(16步)

Put the <box of candy>(31) into the <shopping basket>(30) and put the <receipt>(37) into the <display bin>(2).
inside_<box of candy>(31)_<shopping basket>(30)
inside_<receipt>(37)_<display bin>(2)
)

14、把圆桌子上的钱放到收银台桌子上的basket里面(无人机+狗+臂)(无人机初始在display table2上)
Put the <paper money>(25) into the <shopping basket>(30).
inside_<paper money>(25)_<shopping basket>(30)
无人机起飞,飞向floor,降落,狗走向桌子,抓paper money,走向无人机篮子,放进去,无人机起飞,飞向收银台,降落,机械臂抓paper money,放入篮子(12步)

15、把栅格桌子上的水果apple,饮料等物品分别运到两个圆桌子上(无人机+狗+臂)(无人机初始在display table(5)上)
Put the <apple>(9) on the <pedestal table>(6) and put the <bottle of apple juice>(16) on the <pedestal table>(7).
on_<apple>(9)_<pedestal table>(6)
on_<bottle of apple juice>(16)_<pedestal table>(7)
机械臂抓apple,放入篮子,机械臂抓bottle of apple juice,放入篮子,无人机起飞,飞向floor,降落,机器狗走向篮子,抓apple,走向pedestal table1,放apple,走向篮子,抓bottle of apple juice,走向pedestal table2,放bottle of apple juice(15步)

16、从硬纸盒里拿出apple(44)放入柜台的栅格,拿出的<pack of bread>(27)放入<pedestal table>(6)上(无人机+臂+狗)
Put the <apple>(44) into the <display bin>(2) and put the <pack of bread>(27) on the <pedestal table>(6).
inside_<apple>(44)_<display bin>(2)
on_<pack of bread>(27)_<pedestal table>(6)
GT:机器狗走向carton,抓苹果,走向无人机篮子,放进去,无人机起飞,飞向display table1,降落,机械臂抓apple,放入栅格,狗走向纸盒,抓bread,走向圆桌,放到圆桌上(13步)

17、把checkout table上的巧克力棒放到groceryshelf上,收银台上的paper money放到pedestal table7上(无人机+臂+狗)（无人机初始在柜台上）
Put the <chocolate bar>(35) on the <grocery shelf>(29) and put the <paper money>(36) on the <pedestal table>(7).
on_<chocolate bar>(35)_<grocery shelf>(29)
on_<paper money>(36)_<pedestal table>(7)
GT:机械臂抓巧克力棒,放入篮子,抓paper money,放入篮子,无人机起飞,飞向floor,降落,机器狗走向篮子,抓巧克力棒,走向货架,放巧克力棒,走向篮子,抓paper money,走向桌子,放paper money(15步)

18、把checkout table上basket里面的两盒biscuits放回到货架上(无人机+臂+狗)（无人机初始在柜台上）
Put the <biscuits>(45) and <biscuits>(46) on the <grocery shelf>(29).
on_<biscuits>(45)_<grocery shelf>(29)
on_<biscuits>(46)_<grocery shelf>(29)
GT:机械臂抓biscuits,放入篮子,抓biscuits,放入篮子,无人机起飞,飞向floor,降落,机器狗走向篮子,抓biscuits,走向货架,放biscuits,走向篮子,抓biscuits,走向货架,放biscuits(15步)

19、把display bin(4)中的apple juice放到shelf上(无人机+臂+狗)
Put the <bottle of apple juice>(16) on the <grocery shelf>(29).
on_<bottle of apple juice>(16)_<grocery shelf>(29)
无人机起飞,飞向display table,降落,机械臂抓bottle of apple juice,放入篮子,无人机起飞,飞向floor,降落,机器狗走向篮子,抓bottle of apple juice,走向货架,放bottle of apple juice(12步)

'''


# 环境整理顺序
# 1、先删注释（已完成，2024/1/22）√2024/1/22
# 2、检查一下环境的整体描述relation_<class name>(id)_<class name>(id)是否正确 （已完成，2024/1/22）√2024/1/22
# 2、检查env.py文件里面task的object的id是否正确(已检查21个task的<class name>(id)是与整体环境一致，已完成，2024/1/22) √2024/1/22
# 3、检查object的属性和状态(已完成，2024/1/22) √2024/1/23零点了
# 4、检查任务的GT（检查了task3，5，6，7，10，11，16，17的GT并加了上去，无误，已完成，2024/1/22）√2024/1/23
# 5、把task的谓语逻辑与instruction加上去(八个任务检查无误，已完成，2024/1/22，其他任务待检查) √2024/1/23
# 6、检查task中的初始位置是否正确（八个任务检查无误，已完成，，2024/1/22，其他任务待检查）√2024/1/23
# 7、对task进行删减，包括agent的初始位置的更改(八个任务删减完毕，并且用脚本验证过了，每一个在例表中的子物体，他的父物体也必须在列表中，已完成，2024/1/22，其他任务待检查) √2023/1/23