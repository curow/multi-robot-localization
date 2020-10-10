# How to Generate the CrossRoad Network?
we can generate the network by first specify nodes (cross.nod.xml), edges (cross.edg.xml), type of the edges (cross.typ.xml) and connections between those edges (cross.con.xml),
these four files can then be combined to generate network file (cross.net.xml) by using the following shell command:
```zsh
netconvert --node-files=cross.nod.xml --edge-files=cross.edg.xml --type-files=cross.typ.xml --connection-files=cross.con.xml --output-file=cross.net.xml
```
after the network file is generated, we can use `sumo-gui cross.net.xml` to inspect the generated network.

# Get Random trips in the generated network
the following shell command will do the job (assuming you have specified the $SUMO_HOME variable in shell):
```zsh
python $SUMO_HOME/tools/randomTrips.py -n cross.net.xml -r cross.rou.xml
```
