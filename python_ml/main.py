#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 20 19:13:14 2017

@author: laure
"""
from sklearn.cluster import KMeans
from sklearn.model_selection import train_test_split
import pandas

dataset = pandas.read_csv("../dataset.csv")
X = dataset[1:,:]
y = dataset[1,:]

X_train, X_test= train_test_split(X,y, random_state=42)

kmeans = KMeans(n_clusters=20)
model = kmeans.fit(X)
score = model.score(X)
print(score)