

def deadband(force, threshold):
    newforce = []
    for i in range(len(force)):
        if force[i] < threshold:
            newforce.append(0)
            print(force[i])
        else:
            newforce.append(force[i])
    return newforce

if __name__ == '__main__':
    force1 = [.1, .2, .1, 3, 2, .1]
    
    print(deadband(force1, 2))
    
    
