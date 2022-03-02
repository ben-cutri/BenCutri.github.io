import matplotlib.pyplot as plt

'''
How to use:
1. All files you want to plot should be stored in the data dir
2. Just the name of the path to file from data
(ex: if the path of the file is "data/file.csv" you just need "file.csv"
if the path of the file is "data/date/file.csv" you need "date/file.csv")
3. The interval is in seconds and should be a number. This number can be an integer or a float
(it will be converted into a float either way)
'''


def main():
    plot()


def plot():
    file_name = input('What file would you like to plot?: ')

    f = open('data/' + file_name, 'r')
    r = f.read()

    y = r.split(',')
    interval = y[0]
    y = y[1:]
    y = [int(x) for x in y]

    x = []
    val = 0.0
    for _ in range(len(y)):
        x.append(val)
        val += float(interval)

    plt.plot(x, y)

    x_label = 'Time (' + interval + ' s)'
    plt.xlabel(x_label)

    y_label = 'Analog Value'
    plt.ylabel(y_label)

    title = 'Plot of ' + file_name
    plt.title(title)

    save = input('Do you want to save the plot? (yes/no): ')
    if save.lower() == 'yes' or save.lower() == 'y':
        save_dir = input('Where do you want this data to be stored? (Image will be in data dir): ')
        plt.savefig('data/'+save_dir)

    show = input('Do you want to show the plot? (yes/no): ')
    if show.lower() == 'yes' or show.lower() == 'y':
        plt.show()


if __name__ == '__main__':
    main()
