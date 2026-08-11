# 目标敌人的ID

int32 mode 
# -1:null, 0:armor, 1:small_buff, 2:big_buff, 3:dart


int32 label 
# mode=0时有效，此时0:Sentry, 1:hero, 2:2, 3:3, 4:4, 5:Outpost, 6:Base。其余情况应为-1

# 如果什么都不打，mode=-1且label=-1