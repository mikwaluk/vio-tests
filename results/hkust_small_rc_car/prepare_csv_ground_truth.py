f = open('pseudo_ground_truth.csv', 'r')
line1 = f.readline()
header_line = line1.rstrip().split(',')
second_line = f.readline()
f.close()
second_line_split = second_line.rstrip().split(',')
data = []
data.append(','.join(header_line) + '\n')
i = 0
while i < len(second_line_split) + 1:
    new_line_list = second_line_split[i:i+len(header_line)]
    new_line = ','.join(new_line_list) + '\n'
    data.append(new_line)
    i += len(header_line)

with open('parsed_pseudo_ground_truth.csv', 'w') as f:
    for elem in data:
        f.write(elem)
