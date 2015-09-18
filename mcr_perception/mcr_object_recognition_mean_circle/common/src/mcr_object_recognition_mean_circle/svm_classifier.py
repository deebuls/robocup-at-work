#!/usr/bin/env python

from sklearn.externals import joblib
import numpy as np

class SVMObjectClassifier:
    """
    Defines an SVM classifier with the mean and standard deviation of
    the features, and a label encoder

    """
    def __init__(self, classifier, label_encoder, mean, std):
        self.classifier = classifier
        self.label_encoder = label_encoder
        self.mean = mean
        self.std = std

    def save(self, classifier_name, label_encoder_name):
        joblib.dump(self.classifier, classifier_name)
        joblib.dump([self.label_encoder, self.mean, self.std], label_encoder_name)

    def classify(self, feature_vector):
        feature_vector -= np.array(self.mean)
        feature_vector /= self.std
        probabilities = self.classifier.predict_proba(feature_vector)[0]
        max_index = np.argmax(probabilities)
        cls = self.classifier.classes_[max_index]
        return self.label_encoder.inverse_transform(cls), probabilities[max_index]
    
    @classmethod
    def load(cls, classifier_name, label_encoder_name):
        classifier = joblib.load(classifier_name)
        [label_encoder, mean, std] = joblib.load(label_encoder_name)
        return SVMObjectClassifier(classifier, label_encoder, mean, std)
