#!/usr/bin/python
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

simThetaOffset = 0.0
followThetaDrift = True
if len(sys.argv) > 2:
	followThetaDrift = (int(sys.argv[1]) != 0)
	simThetaOffset = float(sys.argv[2])
	print("set simThetaOffset == "+str(simThetaOffset))
	if followThetaDrift:
		print("will follow theta drift")
	else:
		print("wont follow theta drift")
else:
	print("args:  {follow-theta-drift?}  {simThetaOffset-as-a-multiple-of-2-pi}")

thetadiffcutoffjump = 2.0
thetarealcufoffjump = 3.0

timenow = 0.0
times  = []
thetas = []
omegas = []
cartxs = []
cartvels=[]
sdFs   = []
LQRs   = []
latestThetaOffset = simThetaOffset*6.28318530717958647693

keeplooping = True
fin = open('saved_history_est.txt','r')
while keeplooping:
	linesnums = fin.readline().split()
	if len(linesnums) > 5:
		
		if followThetaDrift and len(thetas) > 2 and abs((thetas[-1]-latestThetaOffset) - float(linesnums[0])) > thetadiffcutoffjump:
			if float(linesnums[0]) > (thetas[-1]-latestThetaOffset):
				latestThetaOffset = (latestThetaOffset - 6.28318530717958647693)
			else:
				latestThetaOffset = (latestThetaOffset + 6.28318530717958647693)
		
		times.append(timenow)
		thetas.append(float(linesnums[0])+latestThetaOffset)
		omegas.append(float(linesnums[1]))
		cartxs.append(float(linesnums[2]))
		cartvels.append(float(linesnums[3]))
		sdFs.append(float(linesnums[4]))
		LQRs.append(float(linesnums[5]))
		
		timenow = (timenow + 0.001)
	else:
		keeplooping = False

latestThetaOffset = 0.0
realthetatimes = []
realthetas = []
keeplooping = True
fin = open('saved_history_real_theta.txt','r')
while keeplooping:
	linesnums = fin.readline().split()
	if len(linesnums) > 1:
		
		if followThetaDrift and len(realthetas) > 2 and abs((realthetas[-1]-latestThetaOffset) - float(linesnums[1])) > thetarealcufoffjump:
			if float(linesnums[1]) > (realthetas[-1]-latestThetaOffset):
				latestThetaOffset = (latestThetaOffset - 6.28318530717958647693)
			else:
				latestThetaOffset = (latestThetaOffset + 6.28318530717958647693)
		
		realthetatimes.append(float(linesnums[0]))
		realthetas.append(float(linesnums[1])+latestThetaOffset)
	else:
		keeplooping = False

realomegatimes = []
realomegas = []
biggestomega = 1.0
keeplooping = True
fin = open('saved_history_real_omega.txt','r')
while keeplooping:
	linesnums = fin.readline().split()
	if len(linesnums) > 1:
		realomegatimes.append(float(linesnums[0]))
		realomegas.append(float(linesnums[1]))
		if abs(float(linesnums[1])) > biggestomega:
			biggestomega = abs(float(linesnums[1]))
	else:
		keeplooping = False
#for i in range(len(realomegas)):
#	realomegas[i] = (realomegas[i] * (6.28 / biggestomega))


realcartxtimes = []
realcartxs = []
keeplooping = True
fin = open('saved_history_real_cartx.txt','r')
while keeplooping:
	linesnums = fin.readline().split()
	if len(linesnums) > 1:
		realcartxtimes.append(float(linesnums[0]))
		realcartxs.append(float(linesnums[1]))
	else:
		keeplooping = False

realcartveltimes = []
realcartvels = []
keeplooping = True
fin = open('saved_history_real_cartvel.txt','r')
while keeplooping:
	linesnums = fin.readline().split()
	if len(linesnums) > 1:
		realcartveltimes.append(float(linesnums[0]))
		realcartvels.append(float(linesnums[1]))
	else:
		keeplooping = False



def PlotVariableAgainstOther(varBlue, varnameBlue, varRed, varnameRed, extraVarX=[], extraVarY=[], extraVarName='', extra2VarX=[], extra2VarY=[], extra2VarName=''):
	fig, ax1 = plt.subplots()
	ax1.plot(times, varBlue, 'b-')
	giventitle=""
	if len(extraVarX) > 0 and len(extraVarY) > 0 and len(extraVarName) > 0:
		giventitle = (giventitle+"extra black pts: "+str(extraVarName))
		ax1.plot(extraVarX, extraVarY, '.k')
	if len(extra2VarX) > 0 and len(extra2VarY) > 0 and len(extra2VarName) > 0:
		giventitle = (giventitle+"extra red pts: "+str(extra2VarName))
		ax1.plot(extra2VarX, extra2VarY, '.r')
	ax1.grid(True,'both')
	ax1.set_title(giventitle)
	ax1.set_xlim([0,1.5])
	ax1.set_xlabel('time (s)')
	ax1.set_ylabel(varnameBlue, color='b')
	for tl in ax1.get_yticklabels():
	    tl.set_color('b')
	if len(varRed) > 0 and len(varnameRed) > 0:
		ax2 = ax1.twinx()
		ax2.plot(times, varRed, 'r')
		ax2.set_ylabel(varnameRed, color='r')
		for tl in ax2.get_yticklabels():
		    tl.set_color('r')

PlotVariableAgainstOther(thetas,'theta (radians)', [],'', realthetatimes, realthetas, 'real thetas')
PlotVariableAgainstOther(omegas,'omega (rad/sec)', [],'', realomegatimes, realomegas, 'real omegas')

PlotVariableAgainstOther(cartxs,'cartxs (m)', [],'', realcartxtimes, realcartxs, 'real cartx')
PlotVariableAgainstOther(cartvels,'cartvels (m)', [],'', realcartveltimes, realcartvels, 'real cartvel')

PlotVariableAgainstOther(cartxs,'cartxs (m)', LQRs, 'LQR PWM')

plt.show()

