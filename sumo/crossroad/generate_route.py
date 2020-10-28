begin = 0
end = 40
number = 20
period = 10
print("<routes>")
# i and j can be 1 to 4
cnt = 0
code = {1 : 'L', 2 : 'R', 3 : 'U', 4 : 'D'}
# for i in reversed(range(1, 4)):
#     for j in reversed(range(1, 4)):
#         if i == j or (i == 4 and j == 1):
#             continue
#         # print(F'\t<flow id="flow{i}{j}" begin="{begin}" end="{end}" from="{i}to0" to="0to{j}" number="{number}" />')
#         # print(F'\t<flow id="{i}{j}-1" begin="{begin + cnt * 3}" end="{end}" from="{i}to0" to="0to{j}" period="{period}" />')
#         print(F'\t<flow id="{code[i]}{code[j]}" begin="{begin + cnt * 2}" end="{end}" from="{i}to0" to="0to{j}" probability="{1/period}" />')
#         cnt += 1
i, j, s, e = 3, 4, 0, 30
print(F'\t<flow id="{code[i]}{code[j]}" begin="{s}" end="{e}" from="{i}to0" to="0to{j}" probability="{1/period}"/>')
print(F'\t<flow id="{code[j]}{code[i]}" begin="{s}" end="{e}" from="{j}to0" to="0to{i}" probability="{1/period}"/>')
i, j, s, e = 1, 2, 10, 20
print(F'\t<flow id="{code[i]}{code[j]}" begin="{10}" end="{20}" from="{i}to0" to="0to{j}" probability="{1/period}"/>')
print(F'\t<flow id="{code[j]}{code[i]}" begin="{s}" end="{e}" from="{j}to0" to="0to{i}" probability="{1/period}"/>')
print("</routes>")
