from sklearn.externals import joblib
from sklearn.neural_network import MLPClassifier
from sklearn.neighbors import KNeighborsClassifier
from sklearn.cluster import KMeans
from sklearn.svm import SVC
from sklearn.model_selection import train_test_split
import pandas



dataset = pandas.read_csv('../dataset_norm.csv', delimiter=',').sort_values(by='move_type')
labels = dataset['move_type']
columns = [item for item in list(dataset.columns) if item!='move_type']
features = dataset[columns]

X_train, X_test, y_train, y_test = train_test_split(features.values, labels.values, test_size=0.33, random_state=42)

knn = KNeighborsClassifier(n_neighbors=4, weights = 'distance')
knn.fit(X_train,y_train)
joblib.dump(knn, 'model_knn.pkl', protocol=2, compress=1) 

kmeans = KMeans(n_clusters=6, random_state=0, max_iter=10000)
kmeans.fit(X_train)
joblib.dump(kmeans, 'model_kmeans.pkl', protocol=2, compress=1) 


mlp = MLPClassifier(solver='lbfgs',
                     hidden_layer_sizes=(50), random_state=1, activation = 'logistic',
                     max_iter = 10000, early_stopping=True)
mlp.fit(X_train,y_train)               
joblib.dump(mlp, 'model_fnn.pkl', protocol=2, compress=1) 

svm = SVC(C=1000, kernel='rbf', gamma = 0.3, probability=True)
svm.fit(X_train, y_train)
joblib.dump(knn, 'model_svm.pkl', protocol=2, compress=1)
