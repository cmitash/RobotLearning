
# coding: utf-8

# In[1]:


import scipy.io as sio
from sklearn.svm import SVR,NuSVR,LinearSVR
from sklearn.linear_model import LinearRegression,Ridge,Lasso,BayesianRidge,ElasticNet,ARDRegression
from sklearn.neighbors import KNeighborsRegressor
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
contents = sio.loadmat('training_data/training_3.mat')
features_3 = contents['features']
scores_3 = contents['score']


# In[3]:

contents = sio.loadmat('test_data/test_4.mat')
features_4 = contents['features']
contents = sio.loadmat('test_data/test_5.mat')
features_5 = contents['features']


# In[4]:

num_examples_1 = len(features_1)
num_examples_2 = len(features_2)
num_examples_3 = len(features_3)
total_number_examples = num_examples_1 + num_examples_2 + num_examples_3

train_features = np.zeros((total_number_examples,25))
train_scores =  np.zeros(total_number_examples)

for index in range(num_examples_1):
    train_features[index] = features_1[index]
for index in range(num_examples_1,num_examples_1+num_examples_2):
    train_features[index] = features_2[index-num_examples_1]
for index in range(num_examples_1+num_examples_2,total_number_examples):
    train_features[index] = features_3[index-(num_examples_1+num_examples_2)]

    
for index in range(num_examples_1):
    train_scores[index] = scores_1[index]
for index in range(num_examples_1,num_examples_1+num_examples_2):
    train_scores[index] = scores_2[index-num_examples_1]
for index in range(num_examples_1+num_examples_2,total_number_examples):
    train_scores[index] = scores_3[index-(num_examples_1+num_examples_2)]


# In[5]:

init_configurations_1 = []
data_file = file('init_data/init_data_4.csv','r')
data = csv.reader(data_file, delimiter=',')
for row in data:
    init_configurations_1.append(row)
    
    
init_configurations_2 = []
data_file = file('init_data/init_data_5.csv','r')
data = csv.reader(data_file, delimiter=',')
for row in data:
    init_configurations_2.append(row)


# In[6]:

classifier = KNeighborsRegressor(n_neighbors=25)
classifier.fit(train_features,train_scores)


# In[7]:

obs_scores = classifier.predict(features_4)
indices = np.argsort(obs_scores)[::-1][:10]


# In[8]:

for val in indices:
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations_1[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'


# In[9]:

obs_scores = classifier.predict(features_5)
indices = np.argsort(obs_scores)[::-1][:10]


# In[10]:

for val in indices:
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations_2[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'


# In[30]:

classifier = SVR()
classifier.fit(train_features,train_scores)


# In[31]:

obs_scores = classifier.predict(features_4)
indices = np.argsort(obs_scores)[::-1][:10]


# In[32]:

for val in indices:
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations_1[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'


# In[33]:

obs_scores = classifier.predict(features_5)
indices = np.argsort(obs_scores)[::-1][:10]


# In[11]:

for val in indices:
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations_2[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'


# In[ ]:

classifier = NuSVR()
classifier.fit(train_features,train_scores)


# In[25]:

obs_scores = classifier.predict(features_4)
indices = np.argsort(obs_scores)[::-1][:10]


# In[27]:

for val in indices:
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations_1[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'


# In[28]:

obs_scores = classifier.predict(features_5)
indices = np.argsort(obs_scores)[::-1][:10]


# In[29]:

for val in indices:
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations_2[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'


# In[6]:

classifier = LinearSVR()
classifier.fit(train_features,train_scores)


# In[7]:

obs_scores = classifier.predict(features_4)
indices = np.argsort(obs_scores)[::-1][:10]


# In[8]:

for val in indices:
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations_1[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'


# In[9]:

obs_scores = classifier.predict(features_5)
indices = np.argsort(obs_scores)[::-1][:10]


# In[10]:

for val in indices:
    print '-'
    print '  relative_config:   [',",".join(map(str,init_configurations_2[val])),']'
    print '  grasp_mode:  2'
    print '  release_mode:  3'


# In[ ]:



