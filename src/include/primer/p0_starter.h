//===----------------------------------------------------------------------===//
//
//                         BusTub
//
// p0_starter.h
//
// Identification: src/include/primer/p0_starter.h
//
// Copyright (c) 2015-2020, Carnegie Mellon University Database Group
//
//===----------------------------------------------------------------------===//

#pragma once

#include <memory>
#include <stdexcept>
#include <vector>

#include "common/exception.h"

namespace bustub {

/**
 * The Matrix type defines a common
 * interface for matrix operations.
 */
template <typename T>
class Matrix {
 protected:
  /**
   * TODO(P0): Add implementation
   *
   * Construct a new Matrix instance.
   * @param rows The number of rows
   * @param cols The number of columns
   *
   */
  Matrix(int rows, int cols) : rows_{rows}, cols_{cols}, linear_{new T[rows * cols]} {}

  /** The number of rows in the matrix */
  int rows_;
  /** The number of columns in the matrix */
  int cols_;

  /**
   * TODO(P0): Allocate the array in the constructor.
   * TODO(P0): Deallocate the array in the destructor.
   * A flattened array containing the elements of the matrix.
   */
  T *linear_;

 public:
  /** @return The number of rows in the matrix */
  virtual auto GetRowCount() const -> int = 0;

  /** @return The number of columns in the matrix */
  virtual auto GetColumnCount() const -> int = 0;

  /**
   * Get the (i,j)th matrix element.
   *
   * Throw OUT_OF_RANGE if either index is out of range.
   *
   * @param i The row index
   * @param j The column index
   * @return The (i,j)th matrix element
   * @throws OUT_OF_RANGE if either index is out of range
   */
  virtual auto GetElement(int i, int j) const -> T = 0;

  /**
   * Set the (i,j)th matrix element.
   *
   * Throw OUT_OF_RANGE if either index is out of range.
   *
   * @param i The row index
   * @param j The column index
   * @param val The value to insert
   * @throws OUT_OF_RANGE if either index is out of range
   */
  virtual void SetElement(int i, int j, T val) = 0;

  /**
   * Fill the elements of the matrix from `source`.
   *
   * Throw OUT_OF_RANGE in the event that `source`
   * does not contain the required number of elements.
   *
   * @param source The source container
   * @throws OUT_OF_RANGE if `source` is incorrect size
   */
  virtual void FillFrom(const std::vector<T> &source) = 0;

  /**
   * Destroy a matrix instance.
   * TODO(P0): Add implementation
   */
  virtual ~Matrix() { delete[] linear_; }
};

/**
 * The RowMatrix type is a concrete matrix implementation.
 * It implements the interface defined by the Matrix type.
 */
template <typename T>
class RowMatrix : public Matrix<T> {
 public:
  /**
   * TODO(P0): Add implementation
   *
   * Construct a new RowMatrix instance.
   * @param rows The number of rows
   * @param cols The number of columns
   */
  RowMatrix(int rows, int cols) : Matrix<T>(rows, cols) {
    data_ = new T *[rows];
    for (auto i = 0; i < rows; i++) {
      data_[i] = &this->linear_[i * cols];
    }
  }

  /**
   * TODO(P0): Add implementation
   * @return The number of rows in the matrix
   */
  int GetRowCount() const override { return this->rows_; }

  /**
   * TODO(P0): Add implementation
   * @return The number of columns in the matrix
   */
  int GetColumnCount() const override { return this->cols_; }

  /**
   * TODO(P0): Add implementation
   *
   * Get the (i,j)th matrix element.
   *
   * Throw OUT_OF_RANGE if either index is out of range.
   *
   * @param i The row index
   * @param j The column index
   * @return The (i,j)th matrix element
   * @throws OUT_OF_RANGE if either index is out of range
   */
  T GetElement(int i, int j) const override {
    if (i < 0 || i >= this->rows_ || j < 0 || j >= this->cols_) {
      throw Exception(ExceptionType::OUT_OF_RANGE, "RowMatrix::GetElement() out of range");
    }
    return data_[i][j];
  }

  /**
   * Set the (i,j)th matrix element.
   *
   * Throw OUT_OF_RANGE if either index is out of range.
   *
   * @param i The row index
   * @param j The column index
   * @param val The value to insert
   * @throws OUT_OF_RANGE if either index is out of range
   */
  void SetElement(int i, int j, T val) override {
    if (i < 0 || i >= this->rows_ || j < 0 || j >= this->cols_) {
      throw Exception(ExceptionType::OUT_OF_RANGE, "RowMatrix::SetElement() out of range");
    }
    data_[i][j] = val;
  }

  /**
   * TODO(P0): Add implementation
   *
   * Fill the elements of the matrix from `source`.
   *
   * Throw OUT_OF_RANGE in the event that `source`
   * does not contain the required number of elements.
   *
   * @param source The source container
   * @throws OUT_OF_RANGE if `source` is incorrect size
   */
  void FillFrom(const std::vector<T> &source) override {
    if (static_cast<int>(source.size()) != this->cols_ * this->rows_) {
      throw Exception(ExceptionType::OUT_OF_RANGE, "RowMatrix::FillFrom() out of range");
    }
    int ind = 0;
    for (int i = 0; i < this->rows_; i++) {
      for (int j = 0; j < this->cols_; j++) {
        data_[i][j] = source[ind++];
      }
    }
  }

  /**
   * TODO(P0): Add implementation
   *
   * Destroy a RowMatrix instance.
   */
  ~RowMatrix() override { delete[] data_; }

 private:
  /**
   * A 2D array containing the elements of the matrix in row-major format.
   *
   * TODO(P0):
   * - Allocate the array of row pointers in the constructor.
   * - Use these pointers to point to corresponding elements of the `linear` array.
   * - Don't forget to deallocate the array in the destructor.
   */
  T **data_;
};

/**
 * The RowMatrixOperations class defines operations
 * that may be performed on instances of `RowMatrix`.
 */
template <typename T>
class RowMatrixOperations {
 public:
  /**
   * Compute (`matrixA` + `matrixB`) and return the result.
   * Return `nullptr` if dimensions mismatch for input matrices.
   * @param matrixA Input matrix
   * @param matrixB Input matrix
   * @return The result of matrix addition
   */
  static auto Add(const RowMatrix<T> *matrixA, const RowMatrix<T> *matrixB) -> std::unique_ptr<RowMatrix<T>> {
    // TODO(P0): Add implementation
    if (!matrixA || !matrixB || matrixA->GetColumnCount() != matrixB->GetColumnCount() ||
        matrixA->GetRowCount() != matrixB->GetRowCount()) {
      return std::unique_ptr<RowMatrix<T>>(nullptr);
    }
    auto *res = new RowMatrix<T>(matrixA->GetRowCount(), matrixA->GetColumnCount());
    for (auto i = 0; i < matrixA->GetRowCount(); i++) {
      for (auto j = 0; j < matrixA->GetColumnCount(); j++) {
        res->SetElement(i, j, matrixA->GetElement(i, j) + matrixB->GetElement(i, j));
      }
    }
    return std::unique_ptr<RowMatrix<T>>(res);
  }

  /**
   * Compute the matrix multiplication (`matrixA` * `matrixB` and return the result.
   * Return `nullptr` if dimensions mismatch for input matrices.  n*m * m*n = n*n
   * @param matrixA Input matrix
   * @param matrixB Input matrix
   * @return The result of matrix multiplication
   */
  static auto Multiply(const RowMatrix<T> *matrixA, const RowMatrix<T> *matrixB) -> std::unique_ptr<RowMatrix<T>> {
    // TODO(P0): Add implementation
    if (!matrixA || !matrixB || matrixA->GetColumnCount() != matrixB->GetRowCount()) {
      return std::unique_ptr<RowMatrix<T>>(nullptr);
    }
    std::unique_ptr<RowMatrix<T>> res_ptr(new RowMatrix<T>(matrixA->GetRowCount(), matrixB->GetColumnCount()));
    for (auto i = 0; i < matrixA->GetRowCount(); i++) {
      for (auto j = 0; j < matrixB->GetColumnCount(); j++) {
        T sum{};
        for (auto k = 0; k < matrixA->GetColumnCount(); k++) {
          sum += matrixA->GetElement(i, k) * matrixB->GetElement(k, j);
        }
        res_ptr->SetElement(i, j, sum);
      }
    }
    return res_ptr;
  }

  /**
   * Simplified General Matrix Multiply operation. Compute (`matrixA` * `matrixB` + `matrixC`).
   * Return `nullptr` if dimensions mismatch for input matrices.
   * @param matrixA Input matrix
   * @param matrixB Input matrix
   * @param matrixC Input matrix
   * @return The result of general matrix multiply
   */
  static auto GEMM(const RowMatrix<T> *matrixA, const RowMatrix<T> *matrixB, const RowMatrix<T> *matrixC)
      -> std::unique_ptr<RowMatrix<T>> {
    // TODO(P0): Add implementation
    return Add(Multiply(matrixA, matrixB), matrixC);
  }
};
}  // namespace bustub
