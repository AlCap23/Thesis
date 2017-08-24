"""
	Contains the Python package for Algorithm used
"""

# Import numpy 
import numpy as np 


# Step Information of SISO system

def Step_Info(y,t, p=0.02, yr = 1):
    """
    Returns the Rise Time, Overshoot, Settling Time and Steady State
    of a given signal. Requires y, yr, t and optional a percentage for settling time
    """

    # Check for Steady State
    # Does series converge?
    if np.abs(y[-1]-yr) < 1e-2 :
        yss = y[-1]
    # If not return error
    else:
        return np.NaN, np.NaN, np.NaN, np.NaN

    # Get the rising time as the time
    # First value above 0.1 steady state
    index1 = np.where(y>=0.1*yss)
    # First value equal 0.9 steady stae
    index2 = np.where(y<=0.9*yss)
    # Rising Time
    # Check if empty
    if index1[0].size == 0:
        t_rise = np.NaN
    elif index2[0].size == 0:
        t_rise = np.NaN
    else:
        t_rise = t[index1[0][0]]-t[index2[0][0]]

    # Overshoot for values above steady state
    # Get all values
    mp = np.abs(y[np.where(abs(y)>abs(yss))])
    # Check if empty
    if mp.size == 0:
        mp = np.NaN
    else:
        mp = np.abs(np.max(mp)-np.abs(yss))


    # Settling time for all value between a certain percentage
    index = np.where(np.logical_and(abs(y)<(1+p)*abs(yss), abs(y)>(1-p)*abs(yss)))
    # Ceck if empty
    if index[0].size ==0:
        t_settle = np.NaN
    else:
        t_settle = t[index[0][0]]


    return t_rise,mp,t_settle,yss



# Integral Identification of first order time delay

def Integral_Identification(y,u,t):
    """Returns a FOTD Model from the given data.
    y - array of outputs
    u - array of inputs
    t - array of time values
    """
    # Truncate for Maximum
    i_end = y.argmax(axis=0)
    yp = y[0:i_end+1]
    up = u[0:i_end+1]
    tp = t[0:i_end+1]
    # Get Gain
    KM = (yp[-1]-yp[0])/(up[-1])
    # Get the Residence Time
    Tar = 1/KM * np.trapz(yp[-1]-yp,tp)
    # Time Constant
    T = np.exp(1)/KM*np.trapz(yp[np.where(tp<=Tar)],tp[np.where(tp<=Tar)])
    # Delay
    L = Tar-T
    # Check if all arguments are valid
    if (T < 0):
        print("Error - Negative lag")
        return -1
    if (L < 0):
        if (L < 1e-2):
            L = 0
        else:    
            print("Error - Negative delay")
            return -1
    return KM,T,L

# Algrotihm for computing gain of first order time delay

def FOTD_Gain(K,T,L,w=0):
    """Computes the gain of a first order time delay system at a given frequency"""
    # Check if all dimensions match
    if (K.shape != T.shape) or (K.shape != L.shape) or (L.shape != T.shape):
        print("Shapes of parameter array are not equal!")
        return np.NaN
    # System Dimension
    if K.ndim == 1:
        # Using system Identity by multiplying with the complex conjugate
        G = 1/(T**2 * w**2 +1)*(K-1j*T*w)*(np.cos(-L*w)+1j*np.sin(-L*w))
    else:
        outputs,inputs = K.shape
        # Create a system within the complex numbers
        G = np.zeros_like(K, dtype=complex)
        for i in range(0,inputs):
            for o in range(0,outputs):
                # Using system Identity by multiplying with the complex conjugate
                G[o][i] = 1 /(T[o][i]**2 * w**2 +1) * ( K[o][i] - 1j*T[o][i]*w) *(np.cos(-L[o][i]*w)+1j*np.sin(-L[o][i]*w))
    return np.real(G)

# Algorithm for computing the RGA

def RGA(K,T,L,w=0):
    """Takes a FOTD System and computes the RGA of the system"""
    if (K.shape != T.shape) or (K.shape != L.shape) or (L.shape != T.shape):
        print("Shapes of parameter array are not equal!")
    # Compute the System
    G = np.absolute(FOTD_Gain(K,T,L,w))
    # Calculate the RGA
    RGA = np.multiply(G, np.transpose(np.linalg.inv(G)))
    return RGA

# Algorithm for AMIGO Tuning

def AMIGO_Tune(K,T,L, structure = 'PI'):
    """Computes the PI(D) controller parameter based on AMIGO algorithm;
       Parameter are returned as parallel notation KP,KI,KD and set point;
       Needs first order time delay parameter as input
    """
    # Check for small delay
    if L < 1e-1:
        L = 1e-1
    # PI Controller
    if structure == 'PI':
        # Parameter as Defined in Aström et. al., Advanced PID Control,p.229 
        KP = 0.15/K + (0.35 - L*T /(L+T)**2)*T/(K*L)
        TI = 0.35*L+(13*L*T**2)/(T**2+12*L*T+7*L**2)
        TD = 0.0
        # Set Point Weight, as given on p.235
        if L/(T+L) > 0.5:
            b = 1
        else:
            b = 0.0
        
    elif structure == 'PID':
        KP = 1/K*(0.2+0.45*T/L)
        TI = (0.4*L + 0.8*T)/(L+0.1*T)*L
        TD = (0.5*L*T)/(0.3*L+T)
        # Set Point Weight, Derived from Fig. 7.2, p. 230
        if L/(T+L) < 0.2:
            b = 0.4
        elif L/(T+L) > 0.3:
            b = 1.0
        else:
            # Approximate as Linear Function
            b = 0.4 + (1.0 - 0.4)/(0.3-0.2)*L/(T+L)
    else:
        print("Undefined controller Structure")
        return np.NaN
    KI = KP/TI
    KD = KP*TD
    return [KP,KI,KD],b

# Algortihm for AMIGO Detuning

def AMIGO_DETUNE(K,T,L,params,KP, MS = 1.4, structure = 'PI'):
    """Detunes the AMIGO parameter according to Astrom"""
    # Check for small delay
    if L < 1e-1:
        L = 1e-1
    # Needed Parameter
    alpha_D = (MS-1)/MS # See p.255 Eq. 7.19
    beta_D = MS*(MS+np.sqrt(MS**2-1))/2# See p.257 Eq. 7.24
    
    # Define old set of parameter
    KP0 = params[0]
    KI0 = params[1]
    KD0 = params[2]
    
    if structure=='PI':
        # Needed constrain for switch case,See p. 258 Eq. 7.27
        c = KP*K - KP0*K*(L+T)/(beta_D*(alpha_D+KP*K)) - alpha_D
        if c < 0:
            KI = beta_D*(alpha_D+KP*K)**2/(K*(L+T))
        else:
            KI = KI0*(alpha_D+KP*K)/(alpha_D+KP0*K)
        return [KP,KP/KI,0.0]
    if structure == 'PID':
        print("Not implemented")
        return np.NaN
    else:
        print("Undefined controller Structure")
        return np.NaN         

# ALgorithm for computing decentralized controller based on RGA

def Control_Decentral(K,T,L, w = 0, b=np.empty, structure = 'PI'):
    """ Computes decentralised controller with AMIGO algorithm based on RGA pairing"""
    # Compute SISO Case
    if K.ndim <= 1:
        # Using system Identity by multiplying with the complex conjugate
        params, b0 = AMIGO_Tune(K,T,L)
        # If b is not given, use b from AMIGO
        if b == np.empty:
            Kr = [b0*params[0], params[1], params[2]]
            Ky = params
        else:
            Kr = [b*params[0],params[1],params[2]]
            Ky = params
    # Compute general MIMO Case
    else:
        # Systems dimensions
        outputs,inputs = K.shape
        # Create an empty controller
        Ky = np.zeros([outputs,inputs,3])
        Kr = np.zeros([outputs,inputs,3])
        # Compute RGA -> Checks for Shape
        LG = RGA(K,T,L,w)
        # Get Pairing as an array for every column
        Pairing = np.argmax(LG, axis=0)
        # Iterate through the pairing
        for o in range(0,outputs):
            # Best Pairing
            i = Pairing[o]
            # Compute controller via recursion
            Ky[o][i],Kr[o][i] = Control_Decentral(K[o][i],T[o][i],L[o][i],b)
    return Ky, Kr

# Algorithm for computing a decoupling control based on Aström

def Control_Astrom(K,T,L,H, MS= None, w = 0, b=np.empty, structure = 'PI'):
    """Computes a Decoupling Controller via Aström Algortihm based on FOTD"""
    
    # Check Input for Maximum Sensitivity
    if MS is None:
        MS = 1.4*np.eye(K.shape[0],K.shape[1])

    # Compute Determinant of Maximum Sensitivity
    ms = np.linalg.det(MS)

    # Compute SISO Case
    if K.ndim <= 1:
        return Control_Decentral(K,T,L,w,b,structure)
    # Compute General MIMO Case
    else:
        
        # Systems dimensions
        outputs,inputs = K.shape
        # Check dimensions
        if (K.shape != T.shape) or (K.shape != H.shape) or (K.shape != MS.shape) or (K.shape != L.shape) or (L.shape != T.shape):
            print("Shapes of parameter array are not equal!")
            return np.NaN
        
        # Create an empty controller
        Ky = np.empty([outputs,inputs,3])
        Kr = np.empty([outputs,inputs,3])
        
        
        # Get minimal Delay/ Time Constant for robust limit of crossover frequency, ignore zeros
        # Important note: wc_min is actually 1/w_c !!!
        if (L[np.where(L>0)].size !=0) or (T[np.where(T>0)].size !=0):
            if (L[np.where(L>0)].size !=0):
                wc_min = np.min([np.min(L[np.nonzero(L)]),np.min(T[np.nonzero(T)])])
            else:
                wc_min = np.min(np.min(T[np.nonzero(T)]))
        else:
            # Use very high frequency
            wc_min = 1e10
        
        # Compute the decoupler
        D = np.linalg.inv(K)
        # Compute the interaction indeces
        # Since d/ds(Q*K) = d/ds(Q)*K = d/ds(G) we can write the Taylor coefficient
        Gamma = np.abs(np.dot(np.multiply(-K,T+L),D))
        # Set main diagonal to zero
        np.fill_diagonal(Gamma,0)
        # Get the maximum of each row 
        GMax = np.argmax(Gamma,axis=0)
        # Iterate through the outputs 
        for o in range(0,outputs):
            # Estimate the new system parameter
            # Get the maximal delay
            l = np.max(L[o][:])
            # Add the systems gain -> scaled
            k = np.dot(K[o][:],D[:][o])
            # Get the system time constant
            t = (np.dot(np.multiply(K[o][:],D[:][o]),T[o][:]+L[o][:]))/k - l
            
            # Design a controller based on estimated system
            ky,kr = Control_Decentral(k,t,l,w,b,structure)
            
            # Test for Interaction
            # We detune the controller of the n-th output in such a way that the maximum of the n-th row is sufficiently small
            # Current maximum interaction
            gmax = Gamma[o][GMax[o]]
            
            # Check for set point weight, either given
            if b == np.empty:
                # Or computed from AMIGO_TUNE
                b = kr[0]/ky[0]
            
            # Check for structure
            if structure == 'PI':
                # Set counter for while
                counter=0
                # Set shrinking rate 
                shrink_rate = 0.9
                while (np.abs(H[o][o]/(ms*gmax)) - np.sqrt( (b*ky[0]/wc_min)**2 + ky[1]**2 ) < 0):
                    if counter > 10:
                        #print('Maximal Iteration for detuning reached! Abort')
                        break
                    # Detune the controller with the shrinking rate    
                    ky = AMIGO_DETUNE(k,t,l,ky,shrink_rate*ky[0])
                    # Increment counter
                    counter += 1
                # Get the controller parameter
                Ky[o][o][:] = ky
                Kr[o][o][:] = [b*ky[0], ky[1], ky[2]]
        
        # Rescale controller for real system
        for c in range(0,2):
            Ky[:][:][c] = np.dot(D,Ky[:][:][c])
            Kr[:][:][c] = np.dot(D,Kr[:][:][c])
        return Ky,Kr


# Modified Detuning
def Control_Decoupled(K,T,L,H, MS= None, w = 0, b=np.empty, structure = 'PI'):
    # Check Input for Maximum Sensitivity
    if MS is None:
        MS = 1.4*np.eye(K.shape[0],K.shape[1])
    
    # Compute Determinant of Maximum Sensitivity
    ms = np.linalg.det(MS)

    # Compute SISO Case
    if K.ndim <= 1:
        return Control_Decentral(K,T,L,w,b,structure)
    
    # Compute General MIMO Case
    else:

        # Compute a decentralized control structure based on RGA
        Ky, Kr = Control_Decentral(K,T,L, w , b, structure)

        # Get minimal Delay/ Time Constant for robust limit of crossover frequency, ignore zeros
        # Important note: wc_min is actually 1/w_c !!!
        if (L[np.where(L>0)].size !=0) or (T[np.where(T>0)].size !=0):
            if (L[np.where(L>0)].size !=0):
                wc_min = np.min([np.min(L[np.nonzero(L)]),np.min(T[np.nonzero(T)])])
            else:
                wc_min = np.min(np.min(T[np.nonzero(T)]))
        else:
            # Use very high frequency
            wc_min = 1e10
        
        # Calculate the Pairing
        # Compute RGA -> Checks for Shape
        LG = RGA(K,T,L,w)
        # Get Pairing as an array for every column
        Pairing = np.argmax(LG, axis=0)
        
        # Compute the Taylor Series 
        Gamma =  np.multiply(-K,T+L)

        # Initialize 
        KD = np.zeros_like(Gamma)
        GD = np.zeros_like(Gamma)

        # Get the Diagonal entries for decoupling
        for outputs in range(0,K.shape[0]):
            inputs = Pairing[outputs]
            KD[outputs][inputs] = K[outputs][inputs]
            GD[outputs][inputs] = Gamma[outputs][inputs]
    
        # Get the Antidiagonal
        KA = K-KD
        GA = Gamma-GD
        
        # Define the splitter
        S = -np.dot(np.linalg.inv(KD),KA)

        # Get the interaction
        GammaA = np.abs(GA + np.dot(GD,S))
        # Get the maximum of each row 
        GMax = np.argmax(GammaA,axis=0)
  
        #Iterate through the outputs
        for outputs in range(0,K.shape[0]):
            inputs = Pairing[outputs]
            
            # Test the current controller for interaction
            # Every controller has the dimension 3 for kp, ki, kd
            ky = Ky[outputs][inputs]
            kr = Kr[outputs][inputs]

            # Get the current parameter
            k = K[outputs][inputs]
            t = T[outputs][inputs]
            l = L[outputs][inputs]

            # Check for set point weight, either given
            if b == np.empty:
                # Or computed from AMIGO_TUNE
                b = kr[0]/ky[0]
            
            gmax = GammaA[outputs][GMax[outputs]]

            # Check for PI Structure
            if structure == 'PI':
                # Define the counter
                counter = 0
                # Set shrinking rate 
                shrink_rate = 0.9
                while (np.abs(H[outputs][outputs]/(ms*gmax)) - np.sqrt( (b*ky[0]/wc_min)**2 + ky[1]**2 ) < 0):
                    if counter > 1e5:
                        #print('Maximal Iteration for detuning reached! Abort')
                        break
                    # Detune the controller with the shrinking rate    
                    ky = AMIGO_DETUNE(k,t,l,ky,shrink_rate*ky[0])
                    # Increment counter
                    counter += 1
                # Get the controller parameter
                Ky[outputs][inputs][:] = ky
                Kr[outputs][inputs][:] = [b*ky[0], ky[1], ky[2]]

        return Ky,Kr