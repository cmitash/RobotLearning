
# coding: utf-8

# In[1]:

import scipy.io as sio
from sklearn.svm import SVR,NuSVR,LinearSVR
from sklearn.linear_model import LinearRegression,Ridge,Lasso,BayesianRidge,ElasticNet,ARDRegression
from sklearn.neighbors import KNeighborsRegressor
from sklearn.kernel_ridge import KernelRidge
import numpy as np
import random,csv
import warnings
warnings.filterwarnings('ignore')


# In[2]:

contents = sio.loadmat('training_data/training_1.mat')
features_1 = contents['features']
scores_1 = contents['score']
contents = sio.loadmat('training_data/training_2.mat')
features_2 = contents['features']
scores_2 = contents['score']


# In[3]:

contents = sio.loadmat('training_data/training_3.mat')
features_3 = contents['features']
# contents = sio.loadmat('test_data/test_4.mat')
# features_4 = contents['features']
# contents = sio.loadmat('test_data/test_5.mat')
# features_5 = contents['features']


# In[4]:

num_examples_1 = len(features_1)
num_examples_2 = len(features_2)
total_number_examples = num_examples_1 + num_examples_2
train_features = np.zeros((total_number_examples,25))
train_scores =  np.zeros(total_number_examples)

for index in range(num_examples_1):
    train_features[index] = features_1[index]
for index in range(num_examples_1,total_number_examples):
    train_features[index] = features_2[index-num_examples_1]

for index in range(num_examples_1):
    train_scores[index] = scores_1[index]
for index in range(num_examples_1,total_number_examples):
    train_scores[index] = scores_2[index-num_examples_1]


# In[5]:

init_configurations = []
data_file = file('init_data/init_data_3.csv','r')
data = csv.reader(data_file, delimiter=',')
for row in data:
    init_configurations.append(row)


# In[59]:

classifier = KNeighborsRegressor(n_neighbors=10)
classifier.fit(train_features,train_scores)


# In[65]:

classifier_2 = KNeighborsRegressor(weights='distance')
classifier_2.fit(train_features,train_scores)


# In[66]:

obs_scores = classifier_2.predict(features_3)
indices = np.argsort(obs_scores)[::-1][:10]


# In[67]:

for val in indices:
    #print val,obs_scores[val],init_configurations[val]
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'    


# In[68]:

classifier = SVR()
classifier.fit(train_features,train_scores)


# In[69]:

obs_scores = classifier.predict(features_3)
indices = np.argsort(obs_scores)[::-1][:10]


# In[70]:

for val in indices:
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'


# In[71]:

classifier = KNeighborsRegressor(n_neighbors=25,weights='distance')
classifier.fit(train_features,train_scores)


# In[72]:

obs_scores = classifier.predict(features_3)
indices = np.argsort(obs_scores)[::-1][:10]


# In[73]:

for val in indices:
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'


# In[ ]:

classifier = SVR()
classifier.fit(train_features,train_scores)


# In[53]:

obs_scores = classifier.predict(features_3)
indices = np.argsort(obs_scores)[::-1][:10]


# In[54]:

for val in indices:
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'


# In[17]:

classifier = NuSVR()
classifier.fit(train_features,train_scores)


# In[84]:

obs_scores = classifier.predict(features_3)
indices = np.argsort(obs_scores)[::-1][:10]


# In[85]:

for val in indices:
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'


# In[8]:

classifier = LinearSVR()
classifier.fit(train_features,train_scores)


# In[9]:

obs_scores = classifier.predict(features_3)
indices = np.argsort(obs_scores)[::-1][:10]


# In[10]:

for val in indices:
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'


# In[11]:

classifier = NuSVR()
classifier.fit(train_features,train_scores)


# In[12]:

obs_scores = classifier.predict(features_3)
indices = np.argsort(obs_scores)[::-1][:10]


# In[13]:

for val in indices:
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'


# In[14]:

classifier = BayesianRidge()
classifier.fit(train_features,train_scores)


# In[15]:

obs_scores = classifier.predict(features_3)
indices = np.argsort(obs_scores)[::-1][:10]


# In[16]:

for val in indices:
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'


# In[18]:

classifier = Ridge()
classifier.fit(train_features,train_scores)


# In[19]:

obs_scores = classifier.predict(features_3)
indices = np.argsort(obs_scores)[::-1][:10]


# In[20]:

for val in indices:
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'


# In[21]:

classifier = Lasso()
classifier.fit(train_features,train_scores)


# In[22]:

obs_scores = classifier.predict(features_3)
indices = np.argsort(obs_scores)[::-1][:10]


# In[23]:

for val in indices:
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'


# In[ ]:



