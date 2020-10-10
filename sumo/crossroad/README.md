# How to Generate the CrossRoad Network?
we can generate the network by first specify nodes (cross.nod.xml), edges (cross.edg.xml), type of the edges (cross.typ.xml) and connections between those edges (cross.con.xml),
these four files can then be combined to generate network file (cross.net.xml) by using the following shell command:
```zsh
netconvert --node-files=cross.nod.xml --edge-files=cross.edg.xml --type-files=cross.typ.xml --connection-files=cross.con.xml --output-file=cross.net.xml
```
after the network file is generated, we can use `sumo-gui cross.net.xml` to inspect the generated network.

# Get Random trips in the generated network
the following shell command will generate random routes (assuming you have specified the $SUMO_HOME variable in shell):
```zsh
python $SUMO_HOME/tools/randomTrips.py -n cross.net.xml -r cross.rou.xml
```

# Or Get Routes using Flows
see the following python script (generate_flow.py):
```python
begin = 0
end = 100
number = 3
print("<routes>")
for i in range(1, 5):
    for j in range(1, 5):
        if i == j:
            continue
        print(F'\t<flow id="flow{i}{j}" begin="{begin}" end="{end}" from="{i}to0" to="0to{j}" number="{number}" />')
print("</routes>")
```
we can use the script to generated flow and then get routes file we needed for the sumo simulation:
```zsh
python generate_flow.py > cross.flow.xml
duarouter --route-files=cross.flow.xml --net=cross.net.xml --output-file=cross.rou.xml
sumo-gui -n cross.net.xml -r cross.rou.xml
```
