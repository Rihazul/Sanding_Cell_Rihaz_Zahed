p1= [2012.95000001, 0.0, 0, 180, 0, 0]
p2= [2012.95000001, 381.000000002, 0, 180, 0, 0]
p3= [0.0, 381.000000002, 0, 180, 0, 0]
p4= [0.0, 0.0, 0, 180, 0, 0]

p9= [704.8500000032, 57.1500000002, -9.525000000039999, 180, 0, 0]
p10= [704.8500000032, 323.8500000012, -9.525000000039999, 180, 0, 0]
p11= [57.1500000002, 323.8500000012, -9.525000000039999, 180, 0, 0]
p12= [57.1500000002, 57.1500000002, -9.525000000039999, 180, 0, 0]

#Conveyer
x=p4[0]
print("x=",x)
increment=p1[0]/5
x1=increment
print("x1=",x1)
x2=increment*2
print("x2=",x2)
x3=increment*3
print("x3=",x3)
x4=increment*4
print("x4=",x4)
x5=increment*5
print("x5=",x5)

#Middle Conveyer movement
xmiddle=p9[0]
print("xmiddle=",xmiddle)

#Offset and Tool Measurement
tool3y = 50.8  # Tool offset in Y
tool3x = 38.1  # Tool offset in X
Offset = 5  # Inner boundary offset

#Bottom Section
point4=[p4[0]+tool3x-9.749,p4[1]+tool3y-19.243,p4[2],p4[3],p4[4],p4[5]]
print("point4=",point4)
point1=[p1[0]/5,p4[1]+tool3y-19.243,p4[2],p4[3],p4[4],p4[5]]
print("point1=",point1)
point41=[p4[0]+tool3x-9.749+1,p4[1]+tool3y-19.243,p4[2],p4[3],p4[4],p4[5]]
print("point41=",point41)
point4pre=[p4[0]+tool3x-9.749,p4[1]+tool3y-19.243,p4[2]-3,p4[3],p4[4],p4[5]]
print("point4pre=",point4pre)
point1pre=[p1[0]/5,p4[1]+tool3y-19.243,p4[2]-3,p4[3],p4[4],p4[5]]
print("point1pre=",point1pre)
point1air=[p1[0]/5,p4[1]+tool3y-19.243,p4[2]-10,p4[3],p4[4],p4[5]]
print("point1air=",point1air)
point4air=[p4[0]+tool3x-9.749,p4[1]+tool3y-19.243,p4[2]-10,p4[3],p4[4],p4[5]]
print("point4air=",point4air)

#Left Side
pointl2=[p4[0]-tool3x+11.138,p4[1]+tool3y-19.243,p4[2]-1,p4[3],p4[4],p4[5]]
print("pointl2=",pointl2)
pointl3=[p4[0]-tool3x+11.138,p3[1]-tool3y+23.769,p4[2]-1,p4[3],p4[4],p4[5]]
print("pointl3=",pointl3)
pointl2pre=[p4[0]-tool3x+11.138,p4[1]+tool3y-19.243,p4[2]-1-3,p4[3],p4[4],p4[5]]
print("pointl2pre=",pointl2pre)
pointl3pre=[p4[0]-tool3x+11.138,p3[1]-tool3y+23.769,p4[2]-1-3,p4[3],p4[4],p4[5]]
print("pointl3pre=",pointl3pre)
pointl3air=[p4[0]-tool3x+11.138,p3[1]-tool3y+23.769,p4[2]-1-3-10,p4[3],p4[4],p4[5]]
print("pointl3air=",pointl3air)
pointl23=[p4[0]-tool3x+11.138,p4[1]+tool3y-19.243+1,p4[2]-1,p4[3],p4[4],p4[5]]
print("pointl23=",pointl23)
#TopCycle
#Top1
pointtop2=[p2[0]/5-32.4,p2[1]-27.04,p2[2]-1,p2[3],p2[4],p2[5]]
print("pointtop2=",pointtop2)
pointtop21=[p2[0]/5-32.4-1,p2[1]-27.04,p2[2]-1,p2[3],p2[4],p2[5]]
print("pointtop21=",pointtop21)
pointtop2pre=[p2[0]/5-32.4,p2[1]-27.04,p2[2]-1-3,p2[3],p2[4],p2[5]]
print("pointtop2pre=",pointtop2pre)
pointtop1=[0,p2[1]-27.04,p2[2]-1,p2[3],p2[4],p2[5]]
print("pointtop1=",pointtop1)
pointtop1pre=[0,p2[1]-27.04,p2[2]-1-3,p2[3],p2[4],p2[5]]
print("pointtop1pre=",pointtop1pre)
pointtop1air=[0,p2[1]-27.04,p2[2]-1-3-6,p2[3],p2[4],p2[5]]
print("pointtop1air=",pointtop1air)
pointtop2air=[p2[0]/5-32.4,p2[1]-27.04,p2[2]-1-9,p2[3],p2[4],p2[5]]
print("pointtop2air=",pointtop2air)

#Right Cycle
point1right=[0+tool3x-9.749,p2[1]-tool3y+23.76,p2[2]-1,p2[3],p2[4],p2[5]]
print("point1right=",point1right)
point1rightpre=[0+tool3x-9.749,p2[1]-tool3y+23.76,p2[2]-1-3,p2[3],p2[4],p2[5]]
print("point1rightpre=",point1rightpre)
point4right=[0+tool3x-9.749,p4[1],p2[2]-1,p2[3],p2[4],p2[5]]
print("point4right=",point4right)
point4rightpre=[0+tool3x-9.749,p4[1],p2[2]-1-3,p2[3],p2[4],p2[5]]
print("point4rightpre=",point4rightpre)
point4rightair=[0+tool3x-9.749,p4[1],p2[2]-1-3-6,p2[3],p2[4],p2[5]]
print("point4rightair=",point4rightair)

#Middle Cycle
point1middle=[p12[0]/2,0+tool3y-20.8,-1,180,0,0]
print("point1middle=",point1middle)
point1middlepre=[p12[0]/2,0+tool3y-20.8,-1-3,180,0,0]
print("point1middlepre=",point1middlepre)
point12middle=[p12[0]/2,0+tool3y-20.8+1,-1,180,0,0]
print("point12middle=",point12middle)
point2middle=[p12[0]/2,p2[1]-tool3y+23.76,-1,180,0,0]
print("point2middle=",point2middle)
point3middle=[((p12[0]/2)*3)+4.28,p2[1]-tool3y+23.76,-1,180,0,0]
print("point3middle=",point3middle)
point4middle=[((p12[0]/2)*3)+4.28,0+tool3y-20.8,-1,180,0,0]
print("point4middle=",point4middle)
point4middlepre=[((p12[0]/2)*3)+4.28,0+tool3y-20.8,-1-3,180,0,0]
print("point4middlepre=",point4middlepre)
point4middleair=[((p12[0]/2)*3)+4.28,0+tool3y-20.8,-1-3-6,180,0,0]
print("point4middleair=",point4middleair)

















