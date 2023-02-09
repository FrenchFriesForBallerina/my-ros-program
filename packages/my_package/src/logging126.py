from csv import writer

def update_csv(lap_time, vehicle_speed, rate, P, I, D):
    #print("writing into file")
    List = [lap_time, vehicle_speed, rate, P, I, D]
    with open('results.csv', 'a') as f_object:
        writer_object = writer(f_object)
        writer_object.writerow(List)
        f_object.close()