# Projection

# Camera index
uint32 camindex

# Point index
uint32 pointindex

# Projection into the image plane
float64 u
float64 v
float64 d

# Is this a stereo projection? (true if stereo, false if monocular)
bool stereo

# Use a covariance matrix?
bool usecovariance

# A 3x3 covariance matrix describing the error
float64[9] covariance

