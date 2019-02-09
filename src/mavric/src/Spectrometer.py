from Spectrometerdriver import Spectrometerdriver

com_port_name = "COM7"
spec = Spectrometerdriver(com_port_name)

spec.reset()
values = spec.scan();
spec.close();

file=open("acq.csv", 'w')
for value in values:
    # write it to a file as text with a comma after it
    file.write(str(value)+",")
    
file.close()
