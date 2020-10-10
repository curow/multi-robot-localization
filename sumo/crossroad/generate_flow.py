begin = 0
end = 300
number = 10
print("<routes>")
for i in range(1, 5):
    for j in range(1, 5):
        if i == j:
            continue
        print(F'\t<flow id="flow{i}{j}" begin="{begin}" end="{end}" from="{i}to0" to="0to{j}" number="{number}" />')
print("</routes>")
