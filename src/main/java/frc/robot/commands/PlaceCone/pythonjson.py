import json
import numpy as np
import matplotlib.pyplot as plt


file='log.json'

with open(file) as json_file:
        data = json.load(json_file)
        result = []
        result2 = []
#print (len(data))

for i in range(len(data)):
    try:
        #print(data[i])
        if data[i]["line"][0:14] == "Distance error":
#            result = float(data[i]["line"][16:19])
            result.append(float(data[i]["line"][16:19]))

        if data[i]["line"][63:74] == "Angle error":
#            result = float(data[i]["line"][16:19])
            result2.append(float(data[i]["line"][76:79]))

#        print(data[i]["line"][0:15])
    except(KeyError):
        continue
    
print(result)
print(result2)
#        print("No line key")


plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True

fig1 = plt.figure("Figure 1")
plt.plot(result, c="red", lw=2)
plt.title("Distance Error")

fig2 = plt.figure("Figure 2")
plt.plot(result2, c="green", lw=5)
plt.title("Angle Error")

plt.xlabel("Time")
plt.ylabel("Error")

#y = result
#x = np.sort(y)

#y2 = result2
#x2 = np.sort(y2)



#plt.title("Line graph")
#plt.plot(x, y, color="red")
#plt.plot(x2, y2, color="green")

plt.show()

    