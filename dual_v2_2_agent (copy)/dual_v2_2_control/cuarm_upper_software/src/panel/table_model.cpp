#include "table_model.h"
#include <iostream>
#include <QStringList>

TabelModel::TabelModel(int row, int col, const QStringList& row_header, const QStringList& col_header, QObject *parent):
    QAbstractTableModel(parent){
    row_ = row;
    col_ = col;
    int rn = row_header.count();
    int cn = col_header.count();
    for (int i=0; i<row; i++){
        QString s = (i < rn)? row_header[i]: "";
        row_header_ << s;
    }
    for (int i=0; i<col; i++){
        QString s = (i < cn)? col_header[i]: "";
        col_header_ << s;
    }
    table_data_ = Eigen::MatrixXf::Zero(row_,col_);
}

// Create a method to populate the model with data
void TabelModel::populateData(const Eigen::MatrixXf&pose){
    int display_decimal = 4;
    float a = pow(10, display_decimal);
    table_data_ = (pose.array() * a).round() / a;
    emit dataChanged(this->index(0,0), this->index(row_-1,col_-1));
}

int TabelModel::rowCount(const QModelIndex &parent) const{
    Q_UNUSED(parent);
    return row_;
}

int TabelModel::columnCount(const QModelIndex &parent) const{
    Q_UNUSED(parent);
    return col_;
}

QVariant TabelModel::data(const QModelIndex &index, int role) const{
    if (!index.isValid() || role != Qt::DisplayRole) {
        return QVariant();
    }
    return table_data_(index.row(), index.column());
}

QVariant TabelModel::headerData(int section, Qt::Orientation orientation, int role) const{
    if (role == Qt::DisplayRole) {
        return (orientation == Qt::Horizontal)? col_header_[section] : row_header_[section];
    }
    return QVariant();
}
