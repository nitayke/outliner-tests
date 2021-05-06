import os

i = 1

for filename in os.listdir("../1_original"):
    print(i)

    os.rename('../1_original/' + filename, '../1_original/' + str(i) + '.jpg')
    i += 1