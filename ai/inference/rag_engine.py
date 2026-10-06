#!/usr/bin/env python3
"""
RAG (Retrieval-Augmented Generation) Engine for HumanCallRobot
Provides vector indexing, semantic search, and document context retrieval
for patient/user voice queries.
"""

import os
import json
import math
from typing import List, Dict, Any

class RAGEngine:
    def __init__(self, docs_dir: str = "docs"):
        self.docs_dir = docs_dir
        self.documents: List[Dict[str, str]] = []
        self.vocabulary: Dict[str, int] = {}
        self.doc_vectors: List[Dict[str, float]] = []
        self._load_and_index_documents()

    def _tokenize(self, text: str) -> List[str]:
        words = text.lower().replace('.', ' ').replace(',', ' ').replace('?', ' ').split()
        return [w for w in words if len(w) > 2]

    def _load_and_index_documents(self):
        """Index all markdown and text documents in docs directory."""
        if not os.path.exists(self.docs_dir):
            os.makedirs(self.docs_dir, exist_ok=True)
            self._create_sample_docs()

        for root, _, files in os.walk(self.docs_dir):
            for file in files:
                if file.endswith((".md", ".txt")):
                    path = os.path.join(root, file)
                    try:
                        with open(path, "r", encoding="utf-8") as f:
                            content = f.read()
                            chunks = self._chunk_text(content, file_name=file)
                            self.documents.extend(chunks)
                    except Exception as e:
                        print(f"Error reading {path}: {e}")

        # Build Term-Frequency index
        for doc in self.documents:
            tokens = self._tokenize(doc["content"])
            tf: Dict[str, float] = {}
            for t in tokens:
                tf[t] = tf.get(t, 0) + 1.0
            total = max(len(tokens), 1)
            for t in tf:
                tf[t] /= total
            self.doc_vectors.append(tf)

    def _chunk_text(self, text: str, file_name: str, chunk_size: int = 300) -> List[Dict[str, str]]:
        paragraphs = text.split("\n\n")
        chunks = []
        for idx, p in enumerate(paragraphs):
            p = p.strip()
            if p:
                chunks.append({
                    "id": f"{file_name}#chunk{idx}",
                    "source": file_name,
                    "content": p
                })
        return chunks

    def query(self, user_query: str, top_k: int = 2) -> Dict[str, Any]:
        """Search for relevant knowledge chunks matching user query."""
        query_tokens = self._tokenize(user_query)
        if not query_tokens:
            return {"query": user_query, "results": [], "answer": "I am here to help. Could you please rephrase your question?"}

        scores = []
        for idx, tf in enumerate(self.doc_vectors):
            score = 0.0
            for t in query_tokens:
                if t in tf:
                    score += tf[t]
            if score > 0:
                scores.append((score, self.documents[idx]))

        scores.sort(key=lambda x: x[0], reverse=True)
        top_results = [item[1] for item in scores[:top_k]]

        if top_results:
            combined_context = " ".join([r["content"] for r in top_results])
            answer = f"Based on facility documentation: {combined_context[:250]}..."
        else:
            answer = "I couldn't find specific information regarding that in my database. Please ask a nurse or staff member."

        return {
            "query": user_query,
            "results": top_results,
            "answer": answer
        }

    def _create_sample_docs(self):
        sample_path = os.path.join(self.docs_dir, "hospital_guide.md")
        with open(sample_path, "w", encoding="utf-8") as f:
            f.write("# Facility & Visiting Guide\n\n"
                    "Visiting hours are from 9:00 AM to 8:00 PM daily. "
                    "Emergency call buttons located near room beds summon the Human Call Robot or nursing team.\n\n"
                    "Room 101, Room 102, and Room 103 are located on Floor 1 along the east wing. "
                    "The Nurse Station is located near the main entrance lobby.\n")

if __name__ == "__main__":
    rag = RAGEngine()
    res = rag.query("Where is Room 101?")
    print("Query Result:", json.dumps(res, indent=2))
