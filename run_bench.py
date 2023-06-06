import os
import statistics

versions = ['baseline','opt']
nAgents = [4000,8000,16000,32000,64000]
initialWorldRadius = 50.
initialQueryRadius = 9.
nSteps = 10
reps = 5

benchmarks = {'const_size': [(a,nSteps,initialWorldRadius,initialQueryRadius) for a in nAgents],
              'const_pop': [(nAgents[0],nSteps,wRadius,initialQueryRadius/initialWorldRadius * wRadius) for wRadius in [x*(initialWorldRadius/nAgents[0]) for x in nAgents]],
              'const_density':[(a,nSteps,wRadius,initialQueryRadius/initialWorldRadius * wRadius) for (a,wRadius) in zip(nAgents,[x*(initialWorldRadius/nAgents[0]) for x in nAgents])]}

OpenSteerPath = ".\\build\\OpenSteerDemo.exe"


for version in versions:
    for (benchmark,configs) in benchmarks.items():
        csvFile = open(f"{version}_{benchmark}.csv","w")
        csvFile.write("Agents;Steps;WorldRadius;QueryRadius;Time(s)\n")
        print(f"Running {benchmark} benchmark, {version} version")
        for config in configs:
            args = f"-no-gui -n {config[0]} -s {config[1]} { '-w '+str(config[2]) if config[2] > initialWorldRadius else ''} {'-q '+str(config[3]) if config[3] > initialQueryRadius else ''} {'-opt' if version == 'opt' else ''}"
            print(f" Agents, Steps, WorldRadius, QueryRadius\n {config}")
            
            completed = False
            while not completed:
                data = []
                csvOut = ""
                for i in range(reps):
                    output = os.popen(OpenSteerPath+" "+args).read().splitlines()[-1]+"\n"
                    data.append(float(output.split(";")[-1]))
                    csvOut += output
                    print(f"   {i+1}/{reps} Completed {data[i]}")
                avg = statistics.mean(data)
                median = statistics.median(data)
                variance = statistics.variance(data)
                print(f"Avg:{avg} Median:{median} Variance:{variance}")
                completed = True 
                if not completed:
                    print("Variance > 5, repeating test")
                else:
                    csvFile.write(csvOut)#.splitlines()[data.index(median)])
                    
            csvFile.write("\n")
         
            

    
#Constant Population Benchmark

#Constant Density Benchmark