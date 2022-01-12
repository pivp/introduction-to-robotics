def summ(A, B):
    for i in range (len(A)):
        for j in range (len(A[i])):
            A[i][j] += B[i][j]
    if len(A[0]) == len(B[0]) and len(A) == len(B):
        return A
    else:
        g = open("output.txt", "w")
        g.write("0")
        g.close()
        quit()  
            
def sub(A, B):
    for i in range (len(A)):
        for j in range (len(A[i])):
            A[i][j] -= B[i][j]
    if len(A[0]) == len(B[0]) and len(A) == len(B):
        return A
    else:
        g = open("output.txt", "w")
        g.write("0")
        g.close()
        quit()              

def mult_num(A, x):
    for i in range (len(A)):
        for j in range(len(A[i])):
            A[i][j] *= x
    return A

def mult_matrix(A, B):
    res = []  
    if (len(A[0]) == len(B)):
        for i in range(len(A)):
            res.append([])
            for j in range(len(B[0])):
                res[i].append(0)
                for k in range(len(A[0])):
                    res[i][j] += A[i][k]*B[k][j]    
        return res
    else:        
        g = open("output.txt", "w")
        g.write("0")
        g.close()
        quit()    

def tran(A):
    res=[]
    n=len(A)
    m=len(A[0])
    for j in range(m):
        tmp=[]
        for i in range(n):
            tmp=tmp+[A[i][j]]
        res=res+[tmp]
    return res

f = open("input.txt", "r")

a, b = map(float,f.readline().split())


#A
na, ma = map(int,f.readline().split())

A = [0] * na
for i in range(na):
    A[i] = [0] * ma
k = 0
nma = f.readline().split()
for i in range(na):
    for j in range(ma):
        A[i][j] = float(nma[k])
        k += 1
        
#B
nb, mb = map(int,f.readline().split())

B = [0] * nb
for i in range(nb):
    B[i] = [0] * mb
k = 0
nmb = f.readline().split()
for i in range(nb):
    for j in range(mb):
        B[i][j] = float(nmb[k])
        k += 1

#C
nc, mc = map(int,f.readline().split())

C = [0] * nc
for i in range(nc):
    C[i] = [0] * mc
k = 0
nmc = f.readline().split()
for i in range(nc):
    for j in range(mc):
        C[i][j] = float(nmc[k])
        k += 1

#D
nd, md = map(int,f.readline().split())

D = [0] * nd
for i in range(nd):
    D[i] = [0] * md
k = 0
nmd = f.readline().split()
for i in range(nd):
    for j in range(md):
        D[i][j] = float(nmd[k])
        k += 1

#F
nf, mf = map(int,f.readline().split())

F = [0] * nf
for i in range(nf):
    F[i] = [0] * mf
k = 0
nmf = f.readline().split()
for i in range(nf):
    for j in range(mf):
        F[i][j] = float(nmf[k])
        k += 1

aA = mult_num(A, a)
BT = tran(B)
bBT = mult_num(BT, b)
AB = summ(aA, bBT)
ABT = tran(AB)
CAB = mult_matrix(C, ABT)
CABD = mult_matrix(CAB, D)
X = sub(CABD, F)

  
line = str(len(X))
columb = str(len(X[0]))
g = open("output.txt", "w")
g.write("1\n")
g.write(line + " ")
g.write(columb + "\n")

for i in range(len(X)):
    line1 = " ".join(map(str, X[i]))
    g.write(line1 + " ")
g.close()

