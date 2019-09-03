import numpy as np
import math

#function  [x,y,utmzone] = deg2utm(Lat,Lon)
def deg2utm(Lat, Lon):
    n1=1#len(Lat)
    n2=1#@len(Lon)
    if (n1!=n2):
        print('Lat and Lon vectors should have the same length');

    # Memory pre-allocation
    x=np.zeros(n1)
    y=np.zeros(n1)
    utmzone = np.chararray(n1)
    utmzone[:]='60 X';

    #% Main Loop
    i = 0
    #%
    la=Lat
    lo=Lon
    sa = 6378137.000000
    sb = 6356752.314245

    sa2 = np.power(sa,2)
    sb2 = np.power(sb,2)


    #e = ( ( ( sa ^ 2 ) - ( sb ^ 2 ) ) ^ 0.5 ) / sa;
    e2 = np.sqrt( sa2 - sb2 ) / sb

    e2cuadrada = np.power(e2, 2)
    c = sa2/ sb;
    #%alpha = ( sa - sb ) / sa;             %f
    #%ablandamiento = 1 / alpha;   % 1/f
    lat = la * ( np.pi / 180 )
    lon = lo * ( np.pi / 180 )

    Huso = round( ( lo / 6 ) + 31)
    S = ( ( Huso * 6 ) - 183 );
    deltaS = lon - ( S * ( np.pi / 180 ) );
    if (la<-72):
        Letra='C'
    elif (la<-64):
        Letra='D'
    elif (la<-56):
        Letra='E'
    elif (la<-48):
        Letra='F'
    elif (la<-40):
        Letra='G'
    elif (la<-32):
        Letra='H'
    elif (la<-24):
        Letra='J'
    elif (la<-16):
        Letra='K'
    elif (la<-8):
        Letra='L';
    elif (la<0):
        Letra='M'
    elif (la<8):
        Letra='N'
    elif (la<16):
        Letra='P'
    elif (la<24):
        Letra='Q'
    elif (la<32):
        Letra='R'
    elif (la<40):
        Letra='S'
    elif (la<48):
        Letra='T';
    elif (la<56):
        Letra='U';
    elif (la<64):
        Letra='V';
    elif (la<72):
        Letra='W';
    else:
         Letra='X';

    a = np.cos(lat) * np.sin(deltaS);
    epsilon = 0.5 * np.log( ( 1 +  a) / ( 1 - a ) )
    nu = math.atan2( np.tan(lat) , np.cos(deltaS) ) - lat
    #v =( c / ( ( 1 + ( e2cuadrada * c  os2 ) ) ) ^ 0.5 ) * 0.9996;
    #v = ( c / ( sqrt1e2cuadradacos2) ) * 0.9996;
    v = ( c / ( np.sqrt( 1 + e2cuadrada*np.power(np.cos(lat), 2)) ) ) * 0.9996;
    ta = ( e2cuadrada / 2 ) * np.power(epsilon,2) * np.power( np.cos(lat), 2);
    a1 = np.sin( 2 * lat );
    a2 = a1 * np.power( np.cos(lat) ,2);
    j2 = lat + ( a1 / 2 );
    j4 = ( ( 3 * j2 ) + a2 ) / 4;
    j6 = ( ( 5 * j4 ) + ( a2 * np.power( np.cos(lat) ,2)) ) / 3;
    alfa = ( 3 / 4 ) * e2cuadrada;
    beta = ( 5 / 3 ) * np.power(alfa,2);
    gama = ( 35 / 27 ) * np.power(alfa,3)
    Bm = 0.9996 * c * ( lat - alfa * j2 + beta * j4 - gama * j6 )
    xx = epsilon * v * ( 1 + ( ta / 3 ) ) + 500000;
    yy = nu * v * ( 1 + ta ) + Bm;
    if (yy<0):
        yy=9999999+yy;
    x[i]=xx
    y[i]=yy;
    utmzone[i] = str(Huso) + Letra
        #sprintf('%02d %c',Huso,Letra);
    return x,y,utmzone
